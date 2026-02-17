# shipshape

Open-source parametric vessel design and validation library.

Shipshape provides boat-design-independent tools for naval engineering analysis. It is used by [Solar Proa](https://github.com/shipshape-marine/solar-proa) but can be applied to any parametric vessel design.

## Modules

| Module | Description | Requires FreeCAD |
|--------|-------------|------------------|
| `parameter` | Merge base parameters and compute derived values via a project-supplied plugin | No |
| `mass` | Compute component masses from a FreeCAD design and material properties | Yes |
| `buoyancy` | Find equilibrium pose (sinkage, pitch, roll) using Newton-Raphson iteration | Yes |
| `gz` | Compute the GZ righting-arm curve over a range of heel angles | Yes |
| `physics` | Center-of-gravity and center-of-buoyancy calculations | Yes |

## Installation

```bash
pip install shipshape
```

For modules that require FreeCAD geometry (mass, buoyancy, gz, physics), install FreeCAD via conda-forge:

```bash
conda install -c conda-forge freecad
```

## CLI Usage

Each module can be run as a CLI tool via `python -m shipshape.<module>`.

### parameter

Merges boat and configuration JSON files, then calls a project-supplied `compute_derived()` function to calculate derived values.

```bash
PYTHONPATH=. python -m shipshape.parameter \
    --compute myproject.parameter.compute \
    --boat constants/boats/boat.json \
    --configuration constants/configurations/config.json \
    --output artifact/parameters.json
```

The `--compute` argument is a dotted module path. The module must export a `compute_derived(data: dict) -> dict` function.

### mass

```bash
python -m shipshape.mass \
    --design artifact/boat.design.FCStd \
    --materials constants/material/materials.json \
    --output artifact/boat.mass.json
```

### buoyancy

```bash
python -m shipshape.buoyancy \
    --design artifact/boat.design.FCStd \
    --materials constants/material/materials.json \
    --output artifact/boat.buoyancy.json
```

### gz

```bash
python -m shipshape.gz \
    --design artifact/boat.design.FCStd \
    --buoyancy artifact/boat.buoyancy.json \
    --output artifact/boat.gz.json \
    --output-png artifact/boat.gz.png
```

## License

Apache 2.0
