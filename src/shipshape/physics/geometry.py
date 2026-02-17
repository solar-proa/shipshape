"""
Geometry abstraction layer for shipshape.

Defines two protocols — GeometryBackend (hot-path geometry ops) and
CadReader (file I/O) — so the rest of the library never imports a
specific CAD kernel directly.

Auto-detection: the first call to get_backend() or get_reader() lazily
imports the FreeCAD backend.  Use set_backend() to override (e.g. for
tests or alternative backends).
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


# "shape" and "doc" are backend-specific opaque objects.
Shape = Any
Doc = Any


@runtime_checkable
class GeometryBackend(Protocol):
    """Hot-path geometry operations (called hundreds of times per solve)."""

    def volume(self, shape: Shape) -> float:
        """Volume in mm³."""
        ...

    def centroid(self, shape: Shape) -> tuple[float, float, float]:
        """Centroid (center of gravity) in mm."""
        ...

    def bounding_box(self, shape: Shape) -> tuple[float, float, float, float, float, float]:
        """(xmin, ymin, zmin, xmax, ymax, zmax) in mm."""
        ...

    def copy(self, shape: Shape) -> Shape:
        """Deep-copy a shape."""
        ...

    def transform(self, shape: Shape, matrix_4x4: list[list[float]]) -> Shape:
        """Apply a 4×4 transformation matrix, returning a *new* shape."""
        ...

    def translate(self, shape: Shape, dx: float, dy: float, dz: float) -> None:
        """Translate a shape in place."""
        ...

    def intersect_with_box(
        self, shape: Shape, origin: tuple[float, float, float], size: tuple[float, float, float]
    ) -> Shape | None:
        """Boolean intersection of *shape* with an axis-aligned box.

        *origin* is (x, y, z) of the box corner with smallest coordinates.
        *size* is (length_x, length_y, length_z).
        Returns None if the result is null or negligible.
        """
        ...


@runtime_checkable
class CadReader(Protocol):
    """File I/O — called once per run."""

    def open(self, path: str) -> Doc:
        """Open a CAD document, return an opaque handle."""
        ...

    def close(self, doc: Doc) -> None:
        """Close a CAD document."""
        ...

    def get_objects(self, doc: Doc) -> list[dict]:
        """Return every object as a dict with keys:

        - "label": str
        - "has_shape": bool
        - "shape": Shape | None  (a *copy*, independent of the document)
        - "global_cog": (x, y, z) | None  (world-frame center of gravity)
        """
        ...


# ---------------------------------------------------------------------------
# Module-level registry
# ---------------------------------------------------------------------------

_backend: GeometryBackend | None = None
_reader: CadReader | None = None


def _auto_detect() -> tuple[GeometryBackend, CadReader]:
    """Try to import known backends in priority order."""
    # FreeCAD
    try:
        from .geometry_freecad import FreeCADBackend, FreeCADReader
        return FreeCADBackend(), FreeCADReader()
    except ImportError:
        pass

    raise RuntimeError(
        "No geometry backend available. Install FreeCAD (conda-forge) "
        "or call set_backend() with a custom implementation."
    )


def set_backend(backend: GeometryBackend, reader: CadReader) -> None:
    """Explicitly set the geometry backend and reader."""
    global _backend, _reader
    _backend = backend
    _reader = reader


def get_backend() -> GeometryBackend:
    """Return the active GeometryBackend, auto-detecting if needed."""
    global _backend, _reader
    if _backend is None:
        _backend, _reader = _auto_detect()
    return _backend


def get_reader() -> CadReader:
    """Return the active CadReader, auto-detecting if needed."""
    global _backend, _reader
    if _reader is None:
        _backend, _reader = _auto_detect()
    return _reader
