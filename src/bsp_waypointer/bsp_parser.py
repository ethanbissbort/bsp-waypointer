"""
BSP Parser Module for Source Engine BSP files (v19-21).

Parses Half-Life 2 / HL2DM map files and extracts geometry and entity data.
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, Dict, List, Optional, Tuple

from .constants import BSPLump, ContentFlags
from .vector import BoundingBox, Plane, Vector3


# BSP file magic number and supported versions
BSP_MAGIC = b"VBSP"
SUPPORTED_VERSIONS = {19, 20, 21}  # HL2, HL2DM, EP1, EP2

# Lump header size
LUMP_HEADER_SIZE = 16
BSP_HEADER_SIZE = 8 + (64 * LUMP_HEADER_SIZE)  # Magic + version + 64 lumps


@dataclass
class LumpInfo:
    """Information about a BSP lump."""
    offset: int
    length: int
    version: int
    fourcc: int


@dataclass
class Face:
    """BSP face structure."""
    plane_index: int
    side: int
    on_node: bool
    first_edge: int
    num_edges: int
    tex_info: int
    disp_info: int
    surface_fog_volume_id: int
    styles: Tuple[int, int, int, int]
    light_offset: int
    area: float
    lightmap_mins: Tuple[int, int]
    lightmap_size: Tuple[int, int]
    orig_face: int
    num_prims: int
    first_prim_id: int
    smoothing_groups: int


@dataclass
class Edge:
    """BSP edge connecting two vertices."""
    v1: int
    v2: int


@dataclass
class Brush:
    """BSP brush structure."""
    first_side: int
    num_sides: int
    contents: int


@dataclass
class BrushSide:
    """BSP brush side structure."""
    plane_index: int
    tex_info: int
    disp_info: int
    bevel: int


@dataclass
class TexInfo:
    """Texture info structure."""
    texture_vecs: Tuple[Tuple[float, ...], Tuple[float, ...]]  # S and T vectors
    lightmap_vecs: Tuple[Tuple[float, ...], Tuple[float, ...]]
    flags: int
    texdata: int


@dataclass
class TexData:
    """Texture data structure."""
    reflectivity: Vector3
    name_string_table_id: int
    width: int
    height: int
    view_width: int
    view_height: int


@dataclass
class Model:
    """BSP model structure (world or brush entity bounds)."""
    mins: Vector3
    maxs: Vector3
    origin: Vector3
    head_node: int
    first_face: int
    num_faces: int


@dataclass
class DispInfo:
    """Displacement info structure."""
    start_position: Vector3
    disp_vert_start: int
    disp_tri_start: int
    power: int
    min_tess: int
    smoothing_angle: float
    contents: int
    map_face: int
    lightmap_alpha_start: int
    lightmap_sample_position_start: int


@dataclass
class DispVert:
    """Displacement vertex."""
    vec: Vector3
    dist: float
    alpha: float


@dataclass
class Entity:
    """Parsed BSP entity with key-value properties."""
    classname: str
    properties: Dict[str, str] = field(default_factory=dict)

    def get(self, key: str, default: str = "") -> str:
        """Get property value with default."""
        return self.properties.get(key, default)

    def get_vector(self, key: str) -> Optional[Vector3]:
        """Parse a vector property (space-separated x y z)."""
        value = self.properties.get(key)
        if not value:
            return None
        parts = value.split()
        if len(parts) >= 3:
            try:
                return Vector3(float(parts[0]), float(parts[1]), float(parts[2]))
            except ValueError:
                return None
        return None

    def get_float(self, key: str, default: float = 0.0) -> float:
        """Get property as float."""
        value = self.properties.get(key)
        if value:
            try:
                return float(value)
            except ValueError:
                pass
        return default

    def get_int(self, key: str, default: int = 0) -> int:
        """Get property as int."""
        value = self.properties.get(key)
        if value:
            try:
                return int(value)
            except ValueError:
                pass
        return default


@dataclass
class BSPFile:
    """Parsed BSP file data."""
    version: int
    path: Path
    lumps: Dict[BSPLump, LumpInfo] = field(default_factory=dict)
    vertices: List[Vector3] = field(default_factory=list)
    planes: List[Plane] = field(default_factory=list)
    faces: List[Face] = field(default_factory=list)
    edges: List[Edge] = field(default_factory=list)
    surfedges: List[int] = field(default_factory=list)
    brushes: List[Brush] = field(default_factory=list)
    brush_sides: List[BrushSide] = field(default_factory=list)
    tex_infos: List[TexInfo] = field(default_factory=list)
    tex_datas: List[TexData] = field(default_factory=list)
    tex_strings: List[str] = field(default_factory=list)
    models: List[Model] = field(default_factory=list)
    disp_infos: List[DispInfo] = field(default_factory=list)
    disp_verts: List[DispVert] = field(default_factory=list)
    entities: List[Entity] = field(default_factory=list)

    @property
    def world_bounds(self) -> Optional[BoundingBox]:
        """Get world geometry bounds from model 0."""
        if self.models:
            m = self.models[0]
            return BoundingBox(m.mins, m.maxs)
        return None

    def get_entities_by_class(self, classname: str) -> List[Entity]:
        """Get all entities with a specific classname."""
        return [e for e in self.entities if e.classname == classname]

    def get_entities_by_prefix(self, prefix: str) -> List[Entity]:
        """Get all entities whose classname starts with a prefix."""
        return [e for e in self.entities if e.classname.startswith(prefix)]


class BSPParser:
    """
    Parser for Source Engine BSP files.

    Supports BSP versions 19-21 (HL2, HL2DM, EP1, EP2).
    """

    def __init__(self):
        self._file: Optional[BinaryIO] = None
        self._bsp: Optional[BSPFile] = None

    def load(self, filepath: str | Path) -> BSPFile:
        """
        Load and parse a BSP file.

        Args:
            filepath: Path to the .bsp file

        Returns:
            Parsed BSPFile object

        Raises:
            ValueError: If file is not a valid BSP or version unsupported
            FileNotFoundError: If file doesn't exist
        """
        filepath = Path(filepath)
        if not filepath.exists():
            raise FileNotFoundError(f"BSP file not found: {filepath}")

        with open(filepath, "rb") as f:
            self._file = f
            self._bsp = BSPFile(version=0, path=filepath)

            self._read_header()
            self._read_all_lumps()

        return self._bsp

    def _read_header(self) -> None:
        """Read and validate BSP header."""
        magic = self._file.read(4)
        if magic != BSP_MAGIC:
            raise ValueError(f"Invalid BSP magic: expected {BSP_MAGIC!r}, got {magic!r}")

        version = struct.unpack("<I", self._file.read(4))[0]
        if version not in SUPPORTED_VERSIONS:
            raise ValueError(
                f"Unsupported BSP version {version}. Supported: {SUPPORTED_VERSIONS}"
            )

        self._bsp.version = version

        # Read lump directory (64 lumps)
        for i in range(64):
            offset, length, version, fourcc = struct.unpack("<IIII", self._file.read(16))
            try:
                lump_type = BSPLump(i)
                self._bsp.lumps[lump_type] = LumpInfo(offset, length, version, fourcc)
            except ValueError:
                # Unknown lump index, skip
                pass

    def _read_all_lumps(self) -> None:
        """Read all relevant lumps."""
        self._read_vertices()
        self._read_planes()
        self._read_edges()
        self._read_surfedges()
        self._read_faces()
        self._read_brushes()
        self._read_brush_sides()
        self._read_texinfo()
        self._read_texdata()
        self._read_texdata_strings()
        self._read_models()
        self._read_dispinfo()
        self._read_dispverts()
        self._read_entities()

    def _read_lump_data(self, lump: BSPLump) -> bytes:
        """Read raw lump data."""
        info = self._bsp.lumps.get(lump)
        if not info or info.length == 0:
            return b""
        self._file.seek(info.offset)
        return self._file.read(info.length)

    def _read_vertices(self) -> None:
        """Read vertex lump (12 bytes per vertex: 3 floats)."""
        data = self._read_lump_data(BSPLump.VERTICES)
        count = len(data) // 12
        for i in range(count):
            x, y, z = struct.unpack_from("<fff", data, i * 12)
            self._bsp.vertices.append(Vector3(x, y, z))

    def _read_planes(self) -> None:
        """Read plane lump (20 bytes per plane)."""
        data = self._read_lump_data(BSPLump.PLANES)
        count = len(data) // 20
        for i in range(count):
            nx, ny, nz, dist, _type = struct.unpack_from("<ffffi", data, i * 20)
            self._bsp.planes.append(Plane(Vector3(nx, ny, nz), dist))

    def _read_edges(self) -> None:
        """Read edge lump (4 bytes per edge: 2 unsigned shorts)."""
        data = self._read_lump_data(BSPLump.EDGES)
        count = len(data) // 4
        for i in range(count):
            v1, v2 = struct.unpack_from("<HH", data, i * 4)
            self._bsp.edges.append(Edge(v1, v2))

    def _read_surfedges(self) -> None:
        """Read surfedge lump (4 bytes per surfedge: signed int)."""
        data = self._read_lump_data(BSPLump.SURFEDGES)
        count = len(data) // 4
        for i in range(count):
            (surfedge,) = struct.unpack_from("<i", data, i * 4)
            self._bsp.surfedges.append(surfedge)

    def _read_faces(self) -> None:
        """Read face lump (56 bytes per face)."""
        data = self._read_lump_data(BSPLump.FACES)
        count = len(data) // 56
        for i in range(count):
            offset = i * 56
            (
                plane_index,
                side,
                on_node,
                first_edge,
                num_edges,
                tex_info,
                disp_info,
                surface_fog_volume_id,
                style0,
                style1,
                style2,
                style3,
                light_offset,
                area,
                lm_min_x,
                lm_min_y,
                lm_size_x,
                lm_size_y,
                orig_face,
                num_prims,
                first_prim_id,
                smoothing_groups,
            ) = struct.unpack_from("<HBBihhhhBBBBifiiiiHHI", data, offset)

            self._bsp.faces.append(
                Face(
                    plane_index=plane_index,
                    side=side,
                    on_node=bool(on_node),
                    first_edge=first_edge,
                    num_edges=num_edges,
                    tex_info=tex_info,
                    disp_info=disp_info,
                    surface_fog_volume_id=surface_fog_volume_id,
                    styles=(style0, style1, style2, style3),
                    light_offset=light_offset,
                    area=area,
                    lightmap_mins=(lm_min_x, lm_min_y),
                    lightmap_size=(lm_size_x, lm_size_y),
                    orig_face=orig_face,
                    num_prims=num_prims,
                    first_prim_id=first_prim_id,
                    smoothing_groups=smoothing_groups,
                )
            )

    def _read_brushes(self) -> None:
        """Read brush lump (12 bytes per brush)."""
        data = self._read_lump_data(BSPLump.BRUSHES)
        count = len(data) // 12
        for i in range(count):
            first_side, num_sides, contents = struct.unpack_from("<iii", data, i * 12)
            self._bsp.brushes.append(Brush(first_side, num_sides, contents))

    def _read_brush_sides(self) -> None:
        """Read brush side lump (8 bytes per side)."""
        data = self._read_lump_data(BSPLump.BRUSHSIDES)
        count = len(data) // 8
        for i in range(count):
            plane_index, tex_info, disp_info, bevel = struct.unpack_from("<Hhhh", data, i * 8)
            self._bsp.brush_sides.append(BrushSide(plane_index, tex_info, disp_info, bevel))

    def _read_texinfo(self) -> None:
        """Read texinfo lump (72 bytes per texinfo)."""
        data = self._read_lump_data(BSPLump.TEXINFO)
        count = len(data) // 72
        for i in range(count):
            offset = i * 72
            # Texture vectors: 2 sets of 4 floats (s and t)
            tex_s = struct.unpack_from("<ffff", data, offset)
            tex_t = struct.unpack_from("<ffff", data, offset + 16)
            # Lightmap vectors: 2 sets of 4 floats
            lm_s = struct.unpack_from("<ffff", data, offset + 32)
            lm_t = struct.unpack_from("<ffff", data, offset + 48)
            flags, texdata = struct.unpack_from("<ii", data, offset + 64)

            self._bsp.tex_infos.append(
                TexInfo(
                    texture_vecs=(tex_s, tex_t),
                    lightmap_vecs=(lm_s, lm_t),
                    flags=flags,
                    texdata=texdata,
                )
            )

    def _read_texdata(self) -> None:
        """Read texdata lump (32 bytes per texdata)."""
        data = self._read_lump_data(BSPLump.TEXDATA)
        count = len(data) // 32
        for i in range(count):
            offset = i * 32
            rx, ry, rz, name_id, width, height, vw, vh = struct.unpack_from(
                "<fffiIIii", data, offset
            )
            self._bsp.tex_datas.append(
                TexData(
                    reflectivity=Vector3(rx, ry, rz),
                    name_string_table_id=name_id,
                    width=width,
                    height=height,
                    view_width=vw,
                    view_height=vh,
                )
            )

    def _read_texdata_strings(self) -> None:
        """Read texture name strings."""
        # Read string table (offsets into string data)
        table_data = self._read_lump_data(BSPLump.TEXDATA_STRING_TABLE)
        string_data = self._read_lump_data(BSPLump.TEXDATA_STRING_DATA)

        if not table_data or not string_data:
            return

        num_strings = len(table_data) // 4
        for i in range(num_strings):
            (offset,) = struct.unpack_from("<I", table_data, i * 4)
            # Read null-terminated string
            end = string_data.find(b"\x00", offset)
            if end == -1:
                end = len(string_data)
            name = string_data[offset:end].decode("ascii", errors="replace")
            self._bsp.tex_strings.append(name)

    def _read_models(self) -> None:
        """Read model lump (48 bytes per model)."""
        data = self._read_lump_data(BSPLump.MODELS)
        count = len(data) // 48
        for i in range(count):
            offset = i * 48
            (
                min_x,
                min_y,
                min_z,
                max_x,
                max_y,
                max_z,
                org_x,
                org_y,
                org_z,
                head_node,
                first_face,
                num_faces,
            ) = struct.unpack_from("<fffffffffiII", data, offset)

            self._bsp.models.append(
                Model(
                    mins=Vector3(min_x, min_y, min_z),
                    maxs=Vector3(max_x, max_y, max_z),
                    origin=Vector3(org_x, org_y, org_z),
                    head_node=head_node,
                    first_face=first_face,
                    num_faces=num_faces,
                )
            )

    def _read_dispinfo(self) -> None:
        """Read displacement info lump."""
        data = self._read_lump_data(BSPLump.DISPINFO)
        if not data:
            return

        # DispInfo is 176 bytes in Source engine
        count = len(data) // 176
        for i in range(count):
            offset = i * 176
            (
                start_x,
                start_y,
                start_z,
                disp_vert_start,
                disp_tri_start,
                power,
                min_tess,
                smoothing_angle,
                contents,
                map_face,
                lm_alpha_start,
                lm_sample_start,
            ) = struct.unpack_from("<fffiiiiifihii", data, offset)

            self._bsp.disp_infos.append(
                DispInfo(
                    start_position=Vector3(start_x, start_y, start_z),
                    disp_vert_start=disp_vert_start,
                    disp_tri_start=disp_tri_start,
                    power=power,
                    min_tess=min_tess,
                    smoothing_angle=smoothing_angle,
                    contents=contents,
                    map_face=map_face,
                    lightmap_alpha_start=lm_alpha_start,
                    lightmap_sample_position_start=lm_sample_start,
                )
            )

    def _read_dispverts(self) -> None:
        """Read displacement vertices lump (20 bytes per vert)."""
        data = self._read_lump_data(BSPLump.DISP_VERTS)
        if not data:
            return

        count = len(data) // 20
        for i in range(count):
            offset = i * 20
            vx, vy, vz, dist, alpha = struct.unpack_from("<fffff", data, offset)
            self._bsp.disp_verts.append(
                DispVert(vec=Vector3(vx, vy, vz), dist=dist, alpha=alpha)
            )

    def _read_entities(self) -> None:
        """Read and parse entity lump."""
        data = self._read_lump_data(BSPLump.ENTITIES)
        if not data:
            return

        # Entity lump is null-terminated text
        text = data.rstrip(b"\x00").decode("ascii", errors="replace")
        self._bsp.entities = self._parse_entities(text)

    def _parse_entities(self, text: str) -> List[Entity]:
        """
        Parse entity lump text into Entity objects.

        Entity format:
        {
        "classname" "info_player_start"
        "origin" "0 0 0"
        ...
        }
        """
        entities = []
        # Pattern to match entity blocks
        entity_pattern = re.compile(r"\{([^}]*)\}", re.DOTALL)
        # Pattern to match key-value pairs
        kv_pattern = re.compile(r'"([^"]+)"\s+"([^"]*)"')

        for match in entity_pattern.finditer(text):
            block = match.group(1)
            properties = {}

            for kv_match in kv_pattern.finditer(block):
                key = kv_match.group(1).lower()  # Normalize to lowercase
                value = kv_match.group(2)
                properties[key] = value

            classname = properties.get("classname", "")
            if classname:
                entities.append(Entity(classname=classname, properties=properties))

        return entities

    def get_face_vertices(self, face: Face) -> List[Vector3]:
        """
        Get the vertices of a face in order.

        Args:
            face: Face structure

        Returns:
            List of vertices in winding order
        """
        vertices = []
        for i in range(face.num_edges):
            surfedge = self._bsp.surfedges[face.first_edge + i]
            if surfedge >= 0:
                edge = self._bsp.edges[surfedge]
                vertices.append(self._bsp.vertices[edge.v1])
            else:
                edge = self._bsp.edges[-surfedge]
                vertices.append(self._bsp.vertices[edge.v2])
        return vertices

    def get_texture_name(self, face: Face) -> str:
        """Get the texture name for a face."""
        if face.tex_info < 0 or face.tex_info >= len(self._bsp.tex_infos):
            return ""
        tex_info = self._bsp.tex_infos[face.tex_info]
        if tex_info.texdata < 0 or tex_info.texdata >= len(self._bsp.tex_datas):
            return ""
        tex_data = self._bsp.tex_datas[tex_info.texdata]
        if tex_data.name_string_table_id >= len(self._bsp.tex_strings):
            return ""
        return self._bsp.tex_strings[tex_data.name_string_table_id]

    def is_solid_brush(self, brush: Brush) -> bool:
        """Check if a brush is solid (walkable)."""
        contents = ContentFlags(brush.contents)
        # Include solid and playerclip, exclude triggers and other non-solid
        return bool(
            contents & (ContentFlags.CONTENTS_SOLID | ContentFlags.CONTENTS_PLAYERCLIP)
        ) and not bool(
            contents
            & (
                ContentFlags.CONTENTS_WATER
                | ContentFlags.CONTENTS_AREAPORTAL
                | ContentFlags.CONTENTS_ORIGIN
            )
        )
