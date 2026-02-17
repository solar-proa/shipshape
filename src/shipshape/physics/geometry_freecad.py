"""
FreeCAD implementation of the geometry abstraction layer.

This is the **only** file in shipshape that imports FreeCAD / Part / Base.
"""

from __future__ import annotations

import sys

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Base
except ImportError as e:
    print(f"ERROR: {e}", file=sys.stderr)
    print("This module requires FreeCAD (conda-forge or bundled)", file=sys.stderr)
    raise


# ---------------------------------------------------------------------------
# GeometryBackend
# ---------------------------------------------------------------------------

class FreeCADBackend:
    """GeometryBackend implemented on top of FreeCAD's Part.Shape."""

    def volume(self, shape: Part.Shape) -> float:
        return shape.Volume

    def centroid(self, shape: Part.Shape) -> tuple[float, float, float]:
        cog = shape.CenterOfGravity
        return (cog.x, cog.y, cog.z)

    def bounding_box(self, shape: Part.Shape) -> tuple[float, float, float, float, float, float]:
        bb = shape.BoundBox
        return (bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax)

    def copy(self, shape: Part.Shape) -> Part.Shape:
        return shape.copy()

    def transform(self, shape: Part.Shape, matrix_4x4: list[list[float]]) -> Part.Shape:
        m = Base.Matrix(
            matrix_4x4[0][0], matrix_4x4[0][1], matrix_4x4[0][2], matrix_4x4[0][3],
            matrix_4x4[1][0], matrix_4x4[1][1], matrix_4x4[1][2], matrix_4x4[1][3],
            matrix_4x4[2][0], matrix_4x4[2][1], matrix_4x4[2][2], matrix_4x4[2][3],
            matrix_4x4[3][0], matrix_4x4[3][1], matrix_4x4[3][2], matrix_4x4[3][3],
        )
        return shape.transformGeometry(m)

    def translate(self, shape: Part.Shape, dx: float, dy: float, dz: float) -> None:
        shape.translate(Base.Vector(dx, dy, dz))

    def intersect_with_box(
        self,
        shape: Part.Shape,
        origin: tuple[float, float, float],
        size: tuple[float, float, float],
    ) -> Part.Shape | None:
        box = Part.makeBox(
            size[0], size[1], size[2],
            Base.Vector(origin[0], origin[1], origin[2]),
        )
        try:
            result = shape.common(box)
        except Exception:
            return None
        if result.isNull() or result.Volume < 1e-6:
            return None
        return result


# ---------------------------------------------------------------------------
# CadReader
# ---------------------------------------------------------------------------

def _get_all_objects(obj_list) -> list:
    """Recursively collect objects including those inside groups."""
    all_objs = []
    for obj in obj_list:
        all_objs.append(obj)
        if hasattr(obj, "Group"):
            all_objs.extend(_get_all_objects(obj.Group))
    return all_objs


def _get_global_cog(obj) -> tuple[float, float, float] | None:
    """Compute the center of gravity in world (global) coordinates.

    Two cases in the design:
    1. Shape created at origin, then positioned via Placement (e.g. Mast)
       — shape bbox center is near origin in X and Y, apply Placement.
    2. Shape created at final world position (e.g. Hull, Ama)
       — shape bbox is already far from origin, use CoG directly.

    We detect case 2 by checking if the bbox center is far from origin.

    Returns None for shapes that don't support CenterOfGravity (e.g.
    shells, wires, or other non-solid geometry).
    """
    try:
        local_cog = obj.Shape.CenterOfGravity
    except RuntimeError:
        return None
    bbox = obj.Shape.BoundBox

    bbox_center_x = (bbox.XMin + bbox.XMax) / 2
    bbox_center_y = (bbox.YMin + bbox.YMax) / 2
    shape_at_origin = abs(bbox_center_x) < 500 and abs(bbox_center_y) < 500

    if shape_at_origin:
        if hasattr(obj, "getGlobalPlacement"):
            global_placement = obj.getGlobalPlacement()
        else:
            global_placement = obj.Placement if hasattr(obj, "Placement") else App.Placement()
        world_cog = global_placement.multVec(local_cog)
        return (world_cog.x, world_cog.y, world_cog.z)
    else:
        return (local_cog.x, local_cog.y, local_cog.z)


class FreeCADReader:
    """CadReader implemented on top of FreeCAD document I/O."""

    def open(self, path: str) -> App.Document:
        return App.openDocument(path)

    def close(self, doc: App.Document) -> None:
        App.closeDocument(doc.Name)

    def get_objects(self, doc: App.Document) -> list[dict]:
        raw = _get_all_objects(doc.Objects)
        result = []
        for obj in raw:
            has_shape = hasattr(obj, "Shape") and not obj.Shape.isNull()
            entry: dict = {
                "label": obj.Label,
                "has_shape": has_shape,
                "shape": obj.Shape.copy() if has_shape else None,
                "global_cog": _get_global_cog(obj) if has_shape else None,
            }
            result.append(entry)
        return result
