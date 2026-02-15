# shipshape

Open-source parametric vessel design and validation library.

## Installation

```
pip install shipshape
```

## Usage

```python
from shipshape.parameter import compute_derived
from shipshape.validate_structure import run_validation

# Compute derived parameters from base parameters
params = compute_derived(base_params)

# Run structural validation
results = run_validation(params, mass_data)
```
