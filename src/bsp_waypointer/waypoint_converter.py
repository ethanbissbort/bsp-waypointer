"""
Waypoint Converter Module for BSP Waypoint Generator.

Converts navigation mesh and entity data to RCBot2 waypoints with
HL2DM-specific flags and metadata.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from .constants import (
    DEFAULT_WAYPOINT_SPACING,
    HL2DMWaypointSubType,
    MAX_CONNECTION_DISTANCE,
    MAX_WAYPOINTS,
    MIN_WAYPOINT_DISTANCE,
    WaypointFlag,
)
from .entity_analyzer import (
    AmmoPickup,
    ArmorItem,
    Breakable,
    Button,
    Charger,
    Door,
    HealthItem,
    HL2DMEntityData,
    Ladder,
    SpawnPoint,
    Teleporter,
    WeaponSpawn,
)
from .geometry_extractor import LadderSurface
from .navmesh_generator import NavigationMesh
from .vector import Vector3


@dataclass
class WaypointMetadata:
    """HL2DM-specific waypoint metadata."""
    subtype: HL2DMWaypointSubType = HL2DMWaypointSubType.SUBTYPE_NONE
    weapon_priority: int = 0
    respawn_time: float = -1.0
    requires_use: bool = False
    entity_origin: Optional[Vector3] = None
    use_position: Optional[Vector3] = None
    target_waypoint: int = -1  # For teleporter pairs


@dataclass
class Waypoint:
    """RCBot2 waypoint structure."""
    index: int
    origin: Vector3
    flags: WaypointFlag = WaypointFlag.W_FL_NONE
    radius: float = 0.0  # Waypoint activation radius
    connections: List[int] = field(default_factory=list)
    metadata: WaypointMetadata = field(default_factory=WaypointMetadata)

    def has_flag(self, flag: WaypointFlag) -> bool:
        """Check if waypoint has a specific flag."""
        return bool(self.flags & flag)

    def add_flag(self, flag: WaypointFlag) -> None:
        """Add a flag to this waypoint."""
        self.flags |= flag

    def remove_flag(self, flag: WaypointFlag) -> None:
        """Remove a flag from this waypoint."""
        self.flags &= ~flag

    def add_connection(self, other_index: int) -> None:
        """Add a connection to another waypoint."""
        if other_index not in self.connections and other_index != self.index:
            self.connections.append(other_index)


class HL2DMWaypointConverter:
    """
    Converts navigation mesh and entities to RCBot2 waypoints.

    Handles placement, connections, and HL2DM-specific flag assignment.
    """

    def __init__(
        self,
        spacing: float = DEFAULT_WAYPOINT_SPACING,
        max_waypoints: int = MAX_WAYPOINTS,
    ):
        """
        Initialize the waypoint converter.

        Args:
            spacing: Distance between waypoints
            max_waypoints: Maximum number of waypoints to generate
        """
        self.spacing = spacing
        self.max_waypoints = max_waypoints
        self._waypoints: List[Waypoint] = []
        self._spatial_hash: Dict[Tuple[int, int, int], List[int]] = {}
        self._cell_size = 128.0

    def convert(
        self,
        navmesh: NavigationMesh,
        entities: HL2DMEntityData,
        ladders: Optional[List[LadderSurface]] = None,
    ) -> List[Waypoint]:
        """
        Convert navigation mesh and entities to waypoints.

        Args:
            navmesh: Navigation mesh
            entities: Parsed HL2DM entities
            ladders: Detected ladder surfaces

        Returns:
            List of waypoints
        """
        self._waypoints = []
        self._spatial_hash = {}

        # Place waypoints on navigation mesh
        self._place_navmesh_waypoints(navmesh)

        # Place waypoints for entities (weapons, items, etc.)
        self._place_weapon_waypoints(entities.weapons)
        self._place_health_waypoints(entities.health_items)
        self._place_armor_waypoints(entities.armor_items)
        self._place_charger_waypoints(entities.chargers)
        self._place_ammo_waypoints(entities.ammo_pickups)
        self._place_spawn_waypoints(entities.spawn_points)
        self._place_teleporter_waypoints(entities.teleporters)
        self._place_ladder_waypoints(entities.ladders, ladders or [])
        self._place_button_waypoints(entities.buttons)
        self._place_breakable_waypoints(entities.breakables)

        # Compute waypoint connections
        self._compute_connections()

        # Assign additional flags based on geometry
        self._assign_geometry_flags(navmesh)

        # Detect sniper positions
        self._detect_sniper_positions()

        # Optimize waypoint count if needed
        if len(self._waypoints) > self.max_waypoints:
            self._optimize_waypoint_count()

        # Reassign indices
        for i, wp in enumerate(self._waypoints):
            wp.index = i

        return self._waypoints

    def _add_waypoint(
        self,
        origin: Vector3,
        flags: WaypointFlag = WaypointFlag.W_FL_NONE,
        metadata: Optional[WaypointMetadata] = None,
        merge_distance: float = MIN_WAYPOINT_DISTANCE,
    ) -> int:
        """
        Add a waypoint, potentially merging with nearby waypoints.

        Args:
            origin: Waypoint position
            flags: Waypoint flags
            metadata: Optional metadata
            merge_distance: Distance for merging nearby waypoints

        Returns:
            Index of the waypoint (new or existing merged)
        """
        # Check for nearby waypoints to merge with
        nearby = self._find_nearby_waypoints(origin, merge_distance)
        if nearby:
            # Merge with nearest existing waypoint
            nearest_idx = nearby[0]
            wp = self._waypoints[nearest_idx]
            wp.flags |= flags
            if metadata and metadata.subtype != HL2DMWaypointSubType.SUBTYPE_NONE:
                # Update metadata if new one has higher priority
                if metadata.weapon_priority > wp.metadata.weapon_priority:
                    wp.metadata = metadata
            return nearest_idx

        # Create new waypoint
        index = len(self._waypoints)
        wp = Waypoint(
            index=index,
            origin=origin,
            flags=flags,
            metadata=metadata or WaypointMetadata(),
        )
        self._waypoints.append(wp)
        self._add_to_spatial_hash(origin, index)

        return index

    def _add_to_spatial_hash(self, origin: Vector3, index: int) -> None:
        """Add waypoint to spatial hash for fast lookup."""
        cx = int(origin.x / self._cell_size)
        cy = int(origin.y / self._cell_size)
        cz = int(origin.z / self._cell_size)
        key = (cx, cy, cz)

        if key not in self._spatial_hash:
            self._spatial_hash[key] = []
        self._spatial_hash[key].append(index)

    def _find_nearby_waypoints(
        self, origin: Vector3, radius: float
    ) -> List[int]:
        """Find waypoints within radius of a point."""
        nearby = []

        cx = int(origin.x / self._cell_size)
        cy = int(origin.y / self._cell_size)
        cz = int(origin.z / self._cell_size)

        # Check nearby cells
        cell_range = int(radius / self._cell_size) + 1
        for dx in range(-cell_range, cell_range + 1):
            for dy in range(-cell_range, cell_range + 1):
                for dz in range(-cell_range, cell_range + 1):
                    key = (cx + dx, cy + dy, cz + dz)
                    if key not in self._spatial_hash:
                        continue

                    for idx in self._spatial_hash[key]:
                        wp = self._waypoints[idx]
                        dist = origin.distance_to(wp.origin)
                        if dist < radius:
                            nearby.append((idx, dist))

        # Sort by distance
        nearby.sort(key=lambda x: x[1])
        return [idx for idx, _ in nearby]

    def _place_navmesh_waypoints(self, navmesh: NavigationMesh) -> None:
        """Place waypoints on navigation mesh polygons."""
        if not navmesh.polygons:
            return

        # Sample points from navmesh
        sample_points = navmesh.sample_points(self.spacing)

        for point in sample_points:
            self._add_waypoint(point)

    def _place_weapon_waypoints(self, weapons: List[WeaponSpawn]) -> None:
        """Place waypoints at weapon spawn locations."""
        for weapon in weapons:
            metadata = WaypointMetadata(
                subtype=weapon.subtype,
                weapon_priority=weapon.priority,
                respawn_time=weapon.respawn_time,
                entity_origin=weapon.origin,
            )
            self._add_waypoint(weapon.origin, WaypointFlag.W_FL_NONE, metadata)

    def _place_health_waypoints(self, health_items: List[HealthItem]) -> None:
        """Place waypoints at health item locations."""
        for item in health_items:
            metadata = WaypointMetadata(
                subtype=item.subtype,
                entity_origin=item.origin,
                requires_use=item.requires_use,
            )
            flags = WaypointFlag.W_FL_HEALTH
            if item.requires_use:
                flags |= WaypointFlag.W_FL_USE
            self._add_waypoint(item.origin, flags, metadata)

    def _place_armor_waypoints(self, armor_items: List[ArmorItem]) -> None:
        """Place waypoints at armor item locations."""
        for item in armor_items:
            metadata = WaypointMetadata(
                subtype=item.subtype,
                entity_origin=item.origin,
                requires_use=item.requires_use,
            )
            flags = WaypointFlag.W_FL_HEALTH  # RCBot2 uses HEALTH for armor too
            if item.requires_use:
                flags |= WaypointFlag.W_FL_USE
            self._add_waypoint(item.origin, flags, metadata)

    def _place_charger_waypoints(self, chargers: List[Charger]) -> None:
        """Place waypoints at charger stations."""
        for charger in chargers:
            # Calculate optimal use position (in front of charger)
            use_offset = charger.facing * 48  # Stand 48 units away
            use_position = charger.origin + use_offset

            metadata = WaypointMetadata(
                subtype=(
                    HL2DMWaypointSubType.ITEM_HEALTHKIT
                    if charger.is_health
                    else HL2DMWaypointSubType.ITEM_BATTERY
                ),
                entity_origin=charger.origin,
                use_position=use_position,
                requires_use=True,
            )
            flags = WaypointFlag.W_FL_HEALTH | WaypointFlag.W_FL_USE
            self._add_waypoint(use_position, flags, metadata)

    def _place_ammo_waypoints(self, ammo: List[AmmoPickup]) -> None:
        """Place waypoints at ammo pickup locations."""
        for pickup in ammo:
            metadata = WaypointMetadata(
                subtype=pickup.subtype,
                entity_origin=pickup.origin,
            )
            self._add_waypoint(pickup.origin, WaypointFlag.W_FL_AMMO, metadata)

    def _place_spawn_waypoints(self, spawns: List[SpawnPoint]) -> None:
        """Place waypoints at player spawn points."""
        for spawn in spawns:
            metadata = WaypointMetadata(
                subtype=HL2DMWaypointSubType.SPAWN_POINT,
                entity_origin=spawn.origin,
            )
            self._add_waypoint(spawn.origin, WaypointFlag.W_FL_NONE, metadata)

    def _place_teleporter_waypoints(self, teleporters: List[Teleporter]) -> None:
        """Place waypoints at teleporter entrances and exits."""
        for teleporter in teleporters:
            # Entrance waypoint
            entrance_meta = WaypointMetadata(
                subtype=HL2DMWaypointSubType.TELEPORT_SOURCE,
                entity_origin=teleporter.entrance_origin,
            )
            entrance_idx = self._add_waypoint(
                teleporter.entrance_origin,
                WaypointFlag.W_FL_TELE_ENTRANCE,
                entrance_meta,
            )

            # Exit waypoint
            exit_meta = WaypointMetadata(
                subtype=HL2DMWaypointSubType.TELEPORT_DEST,
                entity_origin=teleporter.exit_origin,
            )
            exit_idx = self._add_waypoint(
                teleporter.exit_origin,
                WaypointFlag.W_FL_TELE_EXIT,
                exit_meta,
            )

            # Link them
            if entrance_idx < len(self._waypoints) and exit_idx < len(self._waypoints):
                self._waypoints[entrance_idx].metadata.target_waypoint = exit_idx
                self._waypoints[entrance_idx].add_connection(exit_idx)

    def _place_ladder_waypoints(
        self, entity_ladders: List[Ladder], surface_ladders: List[LadderSurface]
    ) -> None:
        """Place waypoints at ladder locations."""
        # From entities
        for ladder in entity_ladders:
            # Place waypoints at bottom and top of ladder
            bottom = Vector3(ladder.origin.x, ladder.origin.y, ladder.mins.z)
            top = Vector3(ladder.origin.x, ladder.origin.y, ladder.maxs.z)

            self._add_waypoint(bottom, WaypointFlag.W_FL_LADDER)
            self._add_waypoint(top, WaypointFlag.W_FL_LADDER)

        # From detected surfaces
        for surface in surface_ladders:
            bottom = Vector3(
                (surface.mins.x + surface.maxs.x) / 2,
                (surface.mins.y + surface.maxs.y) / 2,
                surface.mins.z,
            )
            top = Vector3(
                (surface.mins.x + surface.maxs.x) / 2,
                (surface.mins.y + surface.maxs.y) / 2,
                surface.maxs.z,
            )

            self._add_waypoint(bottom, WaypointFlag.W_FL_LADDER)
            self._add_waypoint(top, WaypointFlag.W_FL_LADDER)

    def _place_button_waypoints(self, buttons: List[Button]) -> None:
        """Place waypoints at button locations."""
        for button in buttons:
            metadata = WaypointMetadata(
                subtype=HL2DMWaypointSubType.BUTTON,
                entity_origin=button.origin,
                requires_use=True,
            )
            self._add_waypoint(button.origin, WaypointFlag.W_FL_USE, metadata)

    def _place_breakable_waypoints(self, breakables: List[Breakable]) -> None:
        """Place waypoints near breakable objects."""
        for breakable in breakables:
            metadata = WaypointMetadata(
                subtype=HL2DMWaypointSubType.BREAKABLE,
                entity_origin=breakable.origin,
            )
            self._add_waypoint(breakable.origin, WaypointFlag.W_FL_BREAKABLE, metadata)

    def _compute_connections(self) -> None:
        """Compute connections between waypoints."""
        for i, wp_a in enumerate(self._waypoints):
            # Find potential connections
            nearby = self._find_nearby_waypoints(wp_a.origin, MAX_CONNECTION_DISTANCE)

            for j in nearby:
                if j == i:
                    continue

                wp_b = self._waypoints[j]

                # Skip if already connected
                if j in wp_a.connections:
                    continue

                # Check connection validity
                if self._can_connect(wp_a, wp_b):
                    wp_a.add_connection(j)
                    wp_b.add_connection(i)

    def _can_connect(self, wp_a: Waypoint, wp_b: Waypoint) -> bool:
        """
        Check if two waypoints can be connected.

        This is a simplified check. Full implementation would use ray tracing.
        """
        distance = wp_a.origin.distance_to(wp_b.origin)

        # Too far
        if distance > MAX_CONNECTION_DISTANCE:
            return False

        # Height difference check (can't connect through floors/ceilings)
        height_diff = abs(wp_a.origin.z - wp_b.origin.z)
        horizontal_dist = wp_a.origin.distance_to_2d(wp_b.origin)

        # Maximum slope check (roughly 60 degrees)
        if horizontal_dist > 0 and height_diff / horizontal_dist > 1.73:
            return False

        # Special connections always allowed (teleporters, ladders)
        if (
            wp_a.has_flag(WaypointFlag.W_FL_TELE_ENTRANCE)
            and wp_b.has_flag(WaypointFlag.W_FL_TELE_EXIT)
        ):
            return True

        if (
            wp_a.has_flag(WaypointFlag.W_FL_LADDER)
            or wp_b.has_flag(WaypointFlag.W_FL_LADDER)
        ):
            return True

        return True

    def _assign_geometry_flags(self, navmesh: NavigationMesh) -> None:
        """Assign flags based on geometry (jump, crouch, fall)."""
        for wp in self._waypoints:
            for conn_idx in wp.connections:
                if conn_idx >= len(self._waypoints):
                    continue

                conn_wp = self._waypoints[conn_idx]
                height_diff = conn_wp.origin.z - wp.origin.z

                # Jump required (going up)
                if 18 < height_diff <= 56:
                    wp.add_flag(WaypointFlag.W_FL_JUMP)

                # Fall warning (going down significantly)
                if height_diff < -200:
                    wp.add_flag(WaypointFlag.W_FL_FALL)

    def _detect_sniper_positions(self) -> None:
        """Detect good sniper/crossbow positions."""
        if len(self._waypoints) < 10:
            return

        # Calculate average height
        avg_z = sum(wp.origin.z for wp in self._waypoints) / len(self._waypoints)

        for wp in self._waypoints:
            # High ground check
            if wp.origin.z < avg_z + 256:
                continue

            # Check for long sight lines
            long_connections = 0
            for conn_idx in wp.connections:
                if conn_idx >= len(self._waypoints):
                    continue
                conn_wp = self._waypoints[conn_idx]
                dist = wp.origin.distance_to(conn_wp.origin)
                if dist > 512:
                    long_connections += 1

            # Has good vantage
            if long_connections >= 2:
                wp.add_flag(WaypointFlag.W_FL_SNIPER)

    def _optimize_waypoint_count(self) -> None:
        """Reduce waypoint count if over maximum."""
        if len(self._waypoints) <= self.max_waypoints:
            return

        # Calculate priority for each waypoint
        priorities = []
        for i, wp in enumerate(self._waypoints):
            priority = 0

            # Entity waypoints are high priority
            if wp.metadata.subtype != HL2DMWaypointSubType.SUBTYPE_NONE:
                priority += 1000
                priority += wp.metadata.weapon_priority

            # Flags add priority
            if wp.has_flag(WaypointFlag.W_FL_HEALTH):
                priority += 500
            if wp.has_flag(WaypointFlag.W_FL_AMMO):
                priority += 300
            if wp.has_flag(WaypointFlag.W_FL_LADDER):
                priority += 800
            if wp.has_flag(WaypointFlag.W_FL_TELE_ENTRANCE):
                priority += 700
            if wp.has_flag(WaypointFlag.W_FL_SNIPER):
                priority += 200

            # Connectivity adds priority
            priority += len(wp.connections) * 10

            priorities.append((i, priority))

        # Sort by priority (descending)
        priorities.sort(key=lambda x: x[1], reverse=True)

        # Keep top waypoints
        keep_indices = set(idx for idx, _ in priorities[: self.max_waypoints])

        # Filter waypoints
        new_waypoints = []
        index_map = {}

        for i, wp in enumerate(self._waypoints):
            if i in keep_indices:
                index_map[i] = len(new_waypoints)
                new_waypoints.append(wp)

        # Update connections
        for wp in new_waypoints:
            wp.connections = [
                index_map[c] for c in wp.connections if c in index_map
            ]
            if wp.metadata.target_waypoint in index_map:
                wp.metadata.target_waypoint = index_map[wp.metadata.target_waypoint]

        self._waypoints = new_waypoints

        # Rebuild spatial hash
        self._spatial_hash = {}
        for i, wp in enumerate(self._waypoints):
            self._add_to_spatial_hash(wp.origin, i)


def convert_to_waypoints(
    navmesh: NavigationMesh,
    entities: HL2DMEntityData,
    ladders: Optional[List[LadderSurface]] = None,
    spacing: float = DEFAULT_WAYPOINT_SPACING,
    max_waypoints: int = MAX_WAYPOINTS,
) -> List[Waypoint]:
    """
    Convenience function to convert navmesh and entities to waypoints.

    Args:
        navmesh: Navigation mesh
        entities: Parsed HL2DM entities
        ladders: Detected ladder surfaces
        spacing: Waypoint spacing
        max_waypoints: Maximum waypoint count

    Returns:
        List of waypoints
    """
    converter = HL2DMWaypointConverter(spacing=spacing, max_waypoints=max_waypoints)
    return converter.convert(navmesh, entities, ladders)
