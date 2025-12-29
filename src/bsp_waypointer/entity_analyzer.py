"""
HL2DM Entity Analyzer Module for BSP Waypoint Generator.

Parses and classifies HL2DM-specific entities including weapons, health,
armor, chargers, ammo, teleporters, and interactables.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .bsp_parser import BSPFile, Entity
from .constants import (
    AMMO_CRATE_MODELS,
    AMMO_ITEMS,
    ARMOR_ITEMS,
    BREAKABLE_ENTITIES,
    BUTTON_ENTITIES,
    CHARGER_ENTITIES,
    DOOR_ENTITIES,
    HEALTH_ITEMS,
    HL2DMWaypointSubType,
    LADDER_ENTITIES,
    SPAWN_ENTITIES,
    TELEPORT_DESTINATION,
    TELEPORT_ENTRANCE,
    WEAPON_DEFINITIONS,
    WaypointFlag,
)
from .vector import BoundingBox, Vector3


@dataclass
class SpawnPoint:
    """Player spawn point."""
    origin: Vector3
    angles: Vector3
    team: str  # "deathmatch", "combine", "rebel"


@dataclass
class WeaponSpawn:
    """Weapon spawn location."""
    origin: Vector3
    classname: str
    subtype: HL2DMWaypointSubType
    priority: int
    respawn_time: float
    entity_name: str = ""


@dataclass
class HealthItem:
    """Health pickup item."""
    origin: Vector3
    classname: str
    subtype: HL2DMWaypointSubType
    priority: int
    is_charger: bool = False
    requires_use: bool = False


@dataclass
class ArmorItem:
    """Armor/battery pickup item."""
    origin: Vector3
    classname: str
    subtype: HL2DMWaypointSubType
    priority: int
    is_charger: bool = False
    requires_use: bool = False


@dataclass
class Charger:
    """Health or armor charger station."""
    origin: Vector3
    mins: Vector3
    maxs: Vector3
    classname: str
    is_health: bool  # True = health charger, False = armor/suit charger
    facing: Vector3  # Direction player should face to use


@dataclass
class AmmoPickup:
    """Ammo pickup item."""
    origin: Vector3
    classname: str
    subtype: HL2DMWaypointSubType
    associated_weapon: str


@dataclass
class AmmoCrate:
    """Ammo crate (bulk dispenser)."""
    origin: Vector3
    crate_type: str  # "ar2", "grenade", "rockets", "smg1"
    model: str


@dataclass
class Teleporter:
    """Teleporter entrance/exit pair."""
    entrance_origin: Vector3
    entrance_mins: Vector3
    entrance_maxs: Vector3
    exit_origin: Vector3
    target_name: str


@dataclass
class Ladder:
    """Ladder entity or surface."""
    origin: Vector3
    mins: Vector3
    maxs: Vector3
    normal: Vector3  # Direction player faces when climbing


@dataclass
class Breakable:
    """Breakable object."""
    origin: Vector3
    mins: Vector3
    maxs: Vector3
    classname: str
    health: int


@dataclass
class Button:
    """Useable button."""
    origin: Vector3
    mins: Vector3
    maxs: Vector3
    target: str  # What it activates


@dataclass
class Door:
    """Door entity."""
    origin: Vector3
    mins: Vector3
    maxs: Vector3
    classname: str
    requires_use: bool  # True if player must press USE


@dataclass
class HL2DMEntityData:
    """Container for all parsed HL2DM entities."""
    spawn_points: List[SpawnPoint] = field(default_factory=list)
    weapons: List[WeaponSpawn] = field(default_factory=list)
    health_items: List[HealthItem] = field(default_factory=list)
    armor_items: List[ArmorItem] = field(default_factory=list)
    chargers: List[Charger] = field(default_factory=list)
    ammo_pickups: List[AmmoPickup] = field(default_factory=list)
    ammo_crates: List[AmmoCrate] = field(default_factory=list)
    teleporters: List[Teleporter] = field(default_factory=list)
    ladders: List[Ladder] = field(default_factory=list)
    breakables: List[Breakable] = field(default_factory=list)
    buttons: List[Button] = field(default_factory=list)
    doors: List[Door] = field(default_factory=list)

    @property
    def total_entities(self) -> int:
        """Total number of parsed game entities."""
        return (
            len(self.spawn_points)
            + len(self.weapons)
            + len(self.health_items)
            + len(self.armor_items)
            + len(self.chargers)
            + len(self.ammo_pickups)
            + len(self.ammo_crates)
            + len(self.teleporters)
            + len(self.ladders)
            + len(self.breakables)
            + len(self.buttons)
            + len(self.doors)
        )


class HL2DMEntityAnalyzer:
    """
    Analyzes BSP entities for HL2DM-specific gameplay elements.

    Identifies weapons, items, chargers, teleporters, and other
    interactive elements for waypoint generation.
    """

    def __init__(self):
        self._bsp: Optional[BSPFile] = None
        self._data: HL2DMEntityData = HL2DMEntityData()
        self._teleport_targets: Dict[str, Vector3] = {}

    def analyze(self, bsp: BSPFile) -> HL2DMEntityData:
        """
        Analyze all entities in a BSP file.

        Args:
            bsp: Parsed BSP file

        Returns:
            Container with all parsed HL2DM entities
        """
        self._bsp = bsp
        self._data = HL2DMEntityData()
        self._teleport_targets = {}

        # First pass: collect teleport destinations
        self._collect_teleport_destinations()

        # Parse all entity types
        self._parse_spawn_points()
        self._parse_weapons()
        self._parse_health_items()
        self._parse_armor_items()
        self._parse_chargers()
        self._parse_ammo()
        self._parse_ammo_crates()
        self._parse_teleporters()
        self._parse_ladders()
        self._parse_breakables()
        self._parse_buttons()
        self._parse_doors()

        return self._data

    def _collect_teleport_destinations(self) -> None:
        """Collect all teleport destination entities."""
        for entity in self._bsp.entities:
            if entity.classname == TELEPORT_DESTINATION:
                name = entity.get("targetname")
                origin = entity.get_vector("origin")
                if name and origin:
                    self._teleport_targets[name] = origin

    def _parse_spawn_points(self) -> None:
        """Parse player spawn point entities."""
        for entity in self._bsp.entities:
            if entity.classname in SPAWN_ENTITIES:
                origin = entity.get_vector("origin")
                if not origin:
                    continue

                angles = entity.get_vector("angles") or Vector3.zero()

                # Determine team based on classname
                if "combine" in entity.classname:
                    team = "combine"
                elif "rebel" in entity.classname:
                    team = "rebel"
                else:
                    team = "deathmatch"

                self._data.spawn_points.append(
                    SpawnPoint(origin=origin, angles=angles, team=team)
                )

    def _parse_weapons(self) -> None:
        """Parse weapon spawn entities."""
        for entity in self._bsp.entities:
            if entity.classname in WEAPON_DEFINITIONS:
                origin = entity.get_vector("origin")
                if not origin:
                    continue

                weapon_info = WEAPON_DEFINITIONS[entity.classname]
                self._data.weapons.append(
                    WeaponSpawn(
                        origin=origin,
                        classname=entity.classname,
                        subtype=weapon_info.subtype,
                        priority=weapon_info.priority,
                        respawn_time=weapon_info.respawn_time,
                        entity_name=entity.get("targetname", ""),
                    )
                )

    def _parse_health_items(self) -> None:
        """Parse health pickup entities."""
        for entity in self._bsp.entities:
            if entity.classname in HEALTH_ITEMS:
                origin = entity.get_vector("origin")
                if not origin:
                    continue

                item_info = HEALTH_ITEMS[entity.classname]
                self._data.health_items.append(
                    HealthItem(
                        origin=origin,
                        classname=entity.classname,
                        subtype=item_info.subtype,
                        priority=item_info.priority,
                    )
                )

    def _parse_armor_items(self) -> None:
        """Parse armor/battery pickup entities."""
        for entity in self._bsp.entities:
            if entity.classname in ARMOR_ITEMS:
                origin = entity.get_vector("origin")
                if not origin:
                    continue

                item_info = ARMOR_ITEMS[entity.classname]
                self._data.armor_items.append(
                    ArmorItem(
                        origin=origin,
                        classname=entity.classname,
                        subtype=item_info.subtype,
                        priority=item_info.priority,
                    )
                )

    def _parse_chargers(self) -> None:
        """Parse health and armor charger stations."""
        for entity in self._bsp.entities:
            if entity.classname in CHARGER_ENTITIES:
                origin = entity.get_vector("origin") or Vector3.zero()
                mins = entity.get_vector("mins") or Vector3(-16, -16, 0)
                maxs = entity.get_vector("maxs") or Vector3(16, 16, 72)

                charger_info = CHARGER_ENTITIES[entity.classname]

                # Calculate facing direction from angles
                angles = entity.get_vector("angles") or Vector3.zero()
                facing = self._angles_to_forward(angles)

                self._data.chargers.append(
                    Charger(
                        origin=origin,
                        mins=mins + origin,
                        maxs=maxs + origin,
                        classname=entity.classname,
                        is_health=charger_info.is_health,
                        facing=facing,
                    )
                )

    def _parse_ammo(self) -> None:
        """Parse ammo pickup entities."""
        for entity in self._bsp.entities:
            if entity.classname in AMMO_ITEMS:
                origin = entity.get_vector("origin")
                if not origin:
                    continue

                ammo_info = AMMO_ITEMS[entity.classname]
                self._data.ammo_pickups.append(
                    AmmoPickup(
                        origin=origin,
                        classname=entity.classname,
                        subtype=ammo_info.subtype,
                        associated_weapon=ammo_info.associated_weapon,
                    )
                )

    def _parse_ammo_crates(self) -> None:
        """Parse ammo crate entities."""
        for entity in self._bsp.entities:
            if entity.classname == "item_ammo_crate":
                origin = entity.get_vector("origin")
                if not origin:
                    continue

                model = entity.get("model", "").lower()
                crate_type = "unknown"

                # Determine crate type from model
                for pattern, type_name in AMMO_CRATE_MODELS.items():
                    if pattern in model:
                        crate_type = type_name
                        break

                self._data.ammo_crates.append(
                    AmmoCrate(origin=origin, crate_type=crate_type, model=model)
                )

    def _parse_teleporters(self) -> None:
        """Parse teleporter entities."""
        for entity in self._bsp.entities:
            if entity.classname == TELEPORT_ENTRANCE:
                origin = entity.get_vector("origin") or Vector3.zero()
                mins = entity.get_vector("mins") or Vector3(-32, -32, 0)
                maxs = entity.get_vector("maxs") or Vector3(32, 32, 72)
                target = entity.get("target", "")

                # Look up destination
                exit_origin = self._teleport_targets.get(target)
                if not exit_origin:
                    continue

                self._data.teleporters.append(
                    Teleporter(
                        entrance_origin=origin,
                        entrance_mins=mins + origin,
                        entrance_maxs=maxs + origin,
                        exit_origin=exit_origin,
                        target_name=target,
                    )
                )

    def _parse_ladders(self) -> None:
        """Parse ladder entities."""
        for entity in self._bsp.entities:
            if entity.classname in LADDER_ENTITIES:
                origin = entity.get_vector("origin") or Vector3.zero()
                mins = entity.get_vector("mins") or Vector3(-16, -16, 0)
                maxs = entity.get_vector("maxs") or Vector3(16, 16, 128)

                # Calculate normal from angles
                angles = entity.get_vector("angles") or Vector3.zero()
                normal = self._angles_to_forward(angles)

                self._data.ladders.append(
                    Ladder(
                        origin=origin,
                        mins=mins + origin,
                        maxs=maxs + origin,
                        normal=normal,
                    )
                )

    def _parse_breakables(self) -> None:
        """Parse breakable entities."""
        for entity in self._bsp.entities:
            if entity.classname in BREAKABLE_ENTITIES:
                origin = entity.get_vector("origin") or Vector3.zero()
                mins = entity.get_vector("mins") or Vector3(-32, -32, 0)
                maxs = entity.get_vector("maxs") or Vector3(32, 32, 72)
                health = entity.get_int("health", 100)

                self._data.breakables.append(
                    Breakable(
                        origin=origin,
                        mins=mins + origin,
                        maxs=maxs + origin,
                        classname=entity.classname,
                        health=health,
                    )
                )

    def _parse_buttons(self) -> None:
        """Parse button entities."""
        for entity in self._bsp.entities:
            if entity.classname in BUTTON_ENTITIES:
                origin = entity.get_vector("origin") or Vector3.zero()
                mins = entity.get_vector("mins") or Vector3(-8, -8, 0)
                maxs = entity.get_vector("maxs") or Vector3(8, 8, 16)
                target = entity.get("target", "")

                self._data.buttons.append(
                    Button(
                        origin=origin,
                        mins=mins + origin,
                        maxs=maxs + origin,
                        target=target,
                    )
                )

    def _parse_doors(self) -> None:
        """Parse door entities."""
        for entity in self._bsp.entities:
            if entity.classname in DOOR_ENTITIES:
                origin = entity.get_vector("origin") or Vector3.zero()
                mins = entity.get_vector("mins") or Vector3(-32, -4, 0)
                maxs = entity.get_vector("maxs") or Vector3(32, 4, 108)

                # Check spawnflags for USE requirement
                spawnflags = entity.get_int("spawnflags", 0)
                requires_use = bool(spawnflags & 256)  # SF_DOOR_USE_OPENS

                self._data.doors.append(
                    Door(
                        origin=origin,
                        mins=mins + origin,
                        maxs=maxs + origin,
                        classname=entity.classname,
                        requires_use=requires_use,
                    )
                )

    def _angles_to_forward(self, angles: Vector3) -> Vector3:
        """Convert Euler angles to forward direction vector."""
        import math

        pitch = math.radians(angles.x)
        yaw = math.radians(angles.y)

        # Forward vector from yaw and pitch
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        return Vector3(cp * cy, cp * sy, -sp)

    # Convenience methods for finding specific entities
    def find_spawn_points(self) -> List[SpawnPoint]:
        """Get all spawn points."""
        return self._data.spawn_points

    def find_weapons(self) -> List[WeaponSpawn]:
        """Get all weapon spawns."""
        return self._data.weapons

    def find_health_items(self) -> List[HealthItem]:
        """Get all health items."""
        return self._data.health_items

    def find_armor_items(self) -> List[ArmorItem]:
        """Get all armor items."""
        return self._data.armor_items

    def find_chargers(self) -> List[Charger]:
        """Get all charger stations."""
        return self._data.chargers

    def find_ammo(self) -> List[AmmoPickup]:
        """Get all ammo pickups."""
        return self._data.ammo_pickups

    def find_ammo_crates(self) -> List[AmmoCrate]:
        """Get all ammo crates."""
        return self._data.ammo_crates

    def find_teleporters(self) -> List[Teleporter]:
        """Get all teleporter pairs."""
        return self._data.teleporters

    def find_ladders(self) -> List[Ladder]:
        """Get all ladders."""
        return self._data.ladders

    def find_breakables(self) -> List[Breakable]:
        """Get all breakable objects."""
        return self._data.breakables

    def find_buttons(self) -> List[Button]:
        """Get all buttons."""
        return self._data.buttons

    def find_doors(self) -> List[Door]:
        """Get all doors."""
        return self._data.doors


def analyze_hl2dm_entities(bsp: BSPFile) -> HL2DMEntityData:
    """
    Convenience function to analyze HL2DM entities.

    Args:
        bsp: Parsed BSP file

    Returns:
        Container with all parsed HL2DM entities
    """
    analyzer = HL2DMEntityAnalyzer()
    return analyzer.analyze(bsp)
