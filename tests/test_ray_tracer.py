"""Tests for BSP ray tracer module."""

import pytest

from bsp_waypointer.ray_tracer import BSPRayTracer, TraceResult
from bsp_waypointer.vector import Plane, Vector3


class MockBSPFile:
    """Mock BSP file for testing ray tracer."""

    def __init__(self):
        self.nodes = []
        self.leafs = []
        self.leaf_brushes = []
        self.brushes = []
        self.brush_sides = []
        self.planes = []


class TestTraceResult:
    """Tests for TraceResult dataclass."""

    def test_default_values(self):
        result = TraceResult(
            hit=False,
            fraction=1.0,
            end_pos=Vector3(0, 0, 0),
        )
        assert result.hit is False
        assert result.fraction == 1.0
        assert result.start_solid is False
        assert result.all_solid is False
        assert result.plane is None

    def test_hit_result(self):
        result = TraceResult(
            hit=True,
            fraction=0.5,
            end_pos=Vector3(5, 5, 5),
            plane=Plane(Vector3(0, 0, 1), 10.0),
            start_solid=False,
        )
        assert result.hit is True
        assert result.fraction == 0.5
        assert result.plane is not None


class TestBSPRayTracer:
    """Tests for BSPRayTracer class."""

    def test_init_with_empty_bsp(self):
        """Test initialization with empty BSP file."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)
        assert tracer.bsp is bsp

    def test_trace_line_empty_bsp(self):
        """Test trace with no BSP tree returns no hit."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(0, 0, 0)
        end = Vector3(100, 100, 100)

        result = tracer.trace_line(start, end)

        assert result.fraction == 1.0
        assert result.hit is False
        assert result.end_pos == end

    def test_line_of_sight_empty_bsp(self):
        """Test line of sight with no BSP tree returns True."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(0, 0, 0)
        end = Vector3(100, 0, 0)

        assert tracer.line_of_sight(start, end) is True

    def test_line_of_sight_with_height_offset(self):
        """Test line of sight applies height offset."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(0, 0, 0)
        end = Vector3(100, 0, 0)

        # Should still work with empty BSP
        assert tracer.line_of_sight(start, end, player_height_offset=36.0) is True

    def test_can_walk_between_empty_bsp(self):
        """Test walk check with no BSP tree returns True."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(0, 0, 0)
        end = Vector3(100, 0, 0)

        assert tracer.can_walk_between(start, end) is True

    def test_trace_hull_empty_bsp(self):
        """Test hull trace with no BSP tree returns no hit."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(0, 0, 0)
        end = Vector3(100, 0, 0)
        mins = Vector3(-16, -16, 0)
        maxs = Vector3(16, 16, 72)

        result = tracer.trace_hull(start, end, mins, maxs)

        assert result.fraction == 1.0
        assert result.hit is False


class TestRayTracerHelpers:
    """Tests for ray tracer helper methods."""

    def test_calc_hull_offset(self):
        """Test hull offset calculation."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        mins = Vector3(-16, -16, 0)
        maxs = Vector3(16, 16, 72)

        # Upward-facing plane
        normal = Vector3(0, 0, 1)
        offset = tracer._calc_hull_offset(normal, mins, maxs)
        assert offset == 0  # Z mins is 0

        # Downward-facing plane
        normal = Vector3(0, 0, -1)
        offset = tracer._calc_hull_offset(normal, mins, maxs)
        assert offset == 72  # Z maxs is 72

        # Side-facing plane
        normal = Vector3(1, 0, 0)
        offset = tracer._calc_hull_offset(normal, mins, maxs)
        assert offset == 16  # X mins is -16, negated = 16


class TestRayTracerIntegration:
    """Integration tests for ray tracer with actual BSP structures."""

    def test_trace_with_simple_node(self):
        """Test trace with a simple BSP node structure."""
        from bsp_waypointer.bsp_parser import BSPNode, BSPLeaf

        bsp = MockBSPFile()

        # Create a simple plane at z=0
        bsp.planes = [Plane(Vector3(0, 0, 1), 0.0)]

        # Create a node that splits at z=0
        # Children: 0 = front (positive z), 1 = back (negative z)
        bsp.nodes = [
            BSPNode(
                plane_index=0,
                children=(-1, -2),  # Both children are leaves (leaf 0 and leaf 1)
                mins=(0, 0, 0),
                maxs=(100, 100, 100),
                first_face=0,
                num_faces=0,
                area=0,
            )
        ]

        # Leaf 0 (front/above plane) - empty
        # Leaf 1 (back/below plane) - solid
        bsp.leafs = [
            BSPLeaf(
                contents=0,  # Empty
                cluster=0,
                area_flags=0,
                mins=(0, 0, 0),
                maxs=(100, 100, 100),
                first_leaf_face=0,
                num_leaf_faces=0,
                first_leaf_brush=0,
                num_leaf_brushes=0,
                leaf_water_data_id=-1,
            ),
            BSPLeaf(
                contents=1,  # CONTENTS_SOLID
                cluster=0,
                area_flags=0,
                mins=(0, 0, -100),
                maxs=(100, 100, 0),
                first_leaf_face=0,
                num_leaf_faces=0,
                first_leaf_brush=0,
                num_leaf_brushes=0,
                leaf_water_data_id=-1,
            ),
        ]

        tracer = BSPRayTracer(bsp)

        # Trace from above to below the plane
        start = Vector3(50, 50, 50)
        end = Vector3(50, 50, -50)

        result = tracer.trace_line(start, end)

        # Should hit the solid leaf
        assert result.hit is True or result.start_solid is True


class TestRayTracerEdgeCases:
    """Test edge cases for ray tracer."""

    def test_zero_length_trace(self):
        """Test trace with same start and end point."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        point = Vector3(50, 50, 50)
        result = tracer.trace_line(point, point)

        assert result.fraction == 1.0

    def test_very_long_trace(self):
        """Test trace over very long distance."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(0, 0, 0)
        end = Vector3(100000, 100000, 100000)

        result = tracer.trace_line(start, end)

        assert result.fraction == 1.0
        assert result.end_pos == end

    def test_negative_coordinates(self):
        """Test trace with negative coordinates."""
        bsp = MockBSPFile()
        tracer = BSPRayTracer(bsp)

        start = Vector3(-100, -100, -100)
        end = Vector3(-200, -200, -200)

        result = tracer.trace_line(start, end)

        assert result.fraction == 1.0
