"""Tests for vector and geometry utilities."""

import math

import pytest

from bsp_waypointer.vector import (
    BoundingBox,
    Plane,
    Triangle,
    Vector3,
    angle_between_vectors,
    point_in_triangle_2d,
)


class TestVector3:
    """Tests for Vector3 class."""

    def test_creation(self):
        v = Vector3(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_zero(self):
        v = Vector3.zero()
        assert v.x == 0.0
        assert v.y == 0.0
        assert v.z == 0.0

    def test_addition(self):
        v1 = Vector3(1, 2, 3)
        v2 = Vector3(4, 5, 6)
        result = v1 + v2
        assert result == Vector3(5, 7, 9)

    def test_subtraction(self):
        v1 = Vector3(4, 5, 6)
        v2 = Vector3(1, 2, 3)
        result = v1 - v2
        assert result == Vector3(3, 3, 3)

    def test_scalar_multiplication(self):
        v = Vector3(1, 2, 3)
        result = v * 2
        assert result == Vector3(2, 4, 6)

    def test_dot_product(self):
        v1 = Vector3(1, 0, 0)
        v2 = Vector3(0, 1, 0)
        assert v1.dot(v2) == 0.0

        v3 = Vector3(1, 0, 0)
        assert v1.dot(v3) == 1.0

    def test_cross_product(self):
        v1 = Vector3(1, 0, 0)
        v2 = Vector3(0, 1, 0)
        result = v1.cross(v2)
        assert result == Vector3(0, 0, 1)

    def test_length(self):
        v = Vector3(3, 4, 0)
        assert v.length() == 5.0

    def test_normalized(self):
        v = Vector3(3, 0, 0)
        n = v.normalized()
        assert abs(n.length() - 1.0) < 1e-6
        assert n == Vector3(1, 0, 0)

    def test_distance_to(self):
        v1 = Vector3(0, 0, 0)
        v2 = Vector3(3, 4, 0)
        assert v1.distance_to(v2) == 5.0

    def test_lerp(self):
        v1 = Vector3(0, 0, 0)
        v2 = Vector3(10, 10, 10)
        result = v1.lerp(v2, 0.5)
        assert result == Vector3(5, 5, 5)


class TestBoundingBox:
    """Tests for BoundingBox class."""

    def test_from_points(self):
        points = [
            Vector3(0, 0, 0),
            Vector3(10, 10, 10),
            Vector3(5, 5, 5),
        ]
        bbox = BoundingBox.from_points(points)
        assert bbox.mins == Vector3(0, 0, 0)
        assert bbox.maxs == Vector3(10, 10, 10)

    def test_center(self):
        bbox = BoundingBox(Vector3(0, 0, 0), Vector3(10, 10, 10))
        center = bbox.center()
        assert center == Vector3(5, 5, 5)

    def test_contains(self):
        bbox = BoundingBox(Vector3(0, 0, 0), Vector3(10, 10, 10))
        assert bbox.contains(Vector3(5, 5, 5))
        assert not bbox.contains(Vector3(15, 5, 5))


class TestTriangle:
    """Tests for Triangle class."""

    def test_normal(self):
        tri = Triangle(
            Vector3(0, 0, 0),
            Vector3(1, 0, 0),
            Vector3(0, 1, 0),
        )
        normal = tri.normal()
        assert abs(normal.z - 1.0) < 1e-6

    def test_center(self):
        tri = Triangle(
            Vector3(0, 0, 0),
            Vector3(3, 0, 0),
            Vector3(0, 3, 0),
        )
        center = tri.center()
        assert abs(center.x - 1.0) < 1e-6
        assert abs(center.y - 1.0) < 1e-6


class TestAngleBetweenVectors:
    """Tests for angle_between_vectors function."""

    def test_perpendicular(self):
        v1 = Vector3(1, 0, 0)
        v2 = Vector3(0, 1, 0)
        angle = angle_between_vectors(v1, v2)
        assert abs(angle - 90.0) < 1e-6

    def test_parallel(self):
        v1 = Vector3(1, 0, 0)
        v2 = Vector3(2, 0, 0)
        angle = angle_between_vectors(v1, v2)
        assert abs(angle) < 1e-6


class TestPointInTriangle:
    """Tests for point_in_triangle_2d function."""

    def test_inside(self):
        v0 = Vector3(0, 0, 0)
        v1 = Vector3(10, 0, 0)
        v2 = Vector3(5, 10, 0)
        point = Vector3(5, 5, 0)
        assert point_in_triangle_2d(point, v0, v1, v2)

    def test_outside(self):
        v0 = Vector3(0, 0, 0)
        v1 = Vector3(10, 0, 0)
        v2 = Vector3(5, 10, 0)
        point = Vector3(20, 20, 0)
        assert not point_in_triangle_2d(point, v0, v1, v2)
