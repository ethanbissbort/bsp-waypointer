"""Tests for constants module."""

import pytest

from bsp_waypointer.constants import (
    AMMO_ITEMS,
    ARMOR_ITEMS,
    CHARGER_ENTITIES,
    DEFAULT_PLAYER_DIMS,
    HEALTH_ITEMS,
    HL2DMWaypointSubType,
    WEAPON_DEFINITIONS,
    WaypointFlag,
)


class TestWaypointFlags:
    """Tests for WaypointFlag enum."""

    def test_flag_values(self):
        assert WaypointFlag.W_FL_NONE == 0
        assert WaypointFlag.W_FL_JUMP == 1
        assert WaypointFlag.W_FL_CROUCH == 2

    def test_flag_combination(self):
        flags = WaypointFlag.W_FL_JUMP | WaypointFlag.W_FL_CROUCH
        assert flags & WaypointFlag.W_FL_JUMP
        assert flags & WaypointFlag.W_FL_CROUCH
        assert not flags & WaypointFlag.W_FL_LADDER


class TestPlayerDimensions:
    """Tests for PlayerDimensions."""

    def test_defaults(self):
        assert DEFAULT_PLAYER_DIMS.standing_height == 72.0
        assert DEFAULT_PLAYER_DIMS.crouch_height == 36.0
        assert DEFAULT_PLAYER_DIMS.radius == 16.0
        assert DEFAULT_PLAYER_DIMS.step_height == 18.0


class TestWeaponDefinitions:
    """Tests for weapon definitions."""

    def test_all_weapons_defined(self):
        expected_weapons = [
            "weapon_rpg",
            "weapon_crossbow",
            "weapon_ar2",
            "weapon_shotgun",
            "weapon_357",
            "weapon_smg1",
            "weapon_physcannon",
            "weapon_frag",
            "weapon_slam",
            "weapon_pistol",
            "weapon_stunstick",
            "weapon_crowbar",
        ]
        for weapon in expected_weapons:
            assert weapon in WEAPON_DEFINITIONS

    def test_weapon_priorities(self):
        # RPG should have highest priority
        assert WEAPON_DEFINITIONS["weapon_rpg"].priority == 95
        # Crowbar should have lowest
        assert WEAPON_DEFINITIONS["weapon_crowbar"].priority == 20

    def test_weapon_subtypes(self):
        assert WEAPON_DEFINITIONS["weapon_rpg"].subtype == HL2DMWaypointSubType.WEAPON_RPG
        assert (
            WEAPON_DEFINITIONS["weapon_crossbow"].subtype
            == HL2DMWaypointSubType.WEAPON_CROSSBOW
        )


class TestItemDefinitions:
    """Tests for item definitions."""

    def test_health_items(self):
        assert "item_healthkit" in HEALTH_ITEMS
        assert "item_healthvial" in HEALTH_ITEMS

    def test_armor_items(self):
        assert "item_battery" in ARMOR_ITEMS
        assert "item_suit" in ARMOR_ITEMS

    def test_ammo_items(self):
        assert "item_ammo_pistol" in AMMO_ITEMS
        assert "item_rpg_round" in AMMO_ITEMS

    def test_charger_entities(self):
        assert "func_healthcharger" in CHARGER_ENTITIES
        assert "func_recharge" in CHARGER_ENTITIES
        assert CHARGER_ENTITIES["func_healthcharger"].is_health is True
        assert CHARGER_ENTITIES["func_recharge"].is_health is False
