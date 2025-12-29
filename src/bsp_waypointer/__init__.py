"""
BSP Waypoint Generator for Half-Life 2: Deathmatch

A standalone tool that parses Source Engine .bsp map files and automatically
generates RCBot2 waypoints for HL2DM without requiring the game engine.
"""

__version__ = "0.1.0"
__author__ = "BSP Waypoint Generator Team"

from .bsp_parser import BSPParser, BSPFile
from .geometry_extractor import GeometryExtractor, TriangleMesh
from .navmesh_generator import NavmeshGenerator, NavigationMesh
from .entity_analyzer import HL2DMEntityAnalyzer, HL2DMEntityData
from .waypoint_converter import HL2DMWaypointConverter, Waypoint
from .rcw_writer import RCWWriter

__all__ = [
    "BSPParser",
    "BSPFile",
    "GeometryExtractor",
    "TriangleMesh",
    "NavmeshGenerator",
    "NavigationMesh",
    "HL2DMEntityAnalyzer",
    "HL2DMEntityData",
    "HL2DMWaypointConverter",
    "Waypoint",
    "RCWWriter",
]
