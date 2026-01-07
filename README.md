# DynamicPlanning.jl

[![SciML Code Style](https://img.shields.io/static/v1?label=code%20style&message=SciML&color=9558b2&labelColor=389826)](https://github.com/SciML/SciMLStyle)
[![Build Status](https://github.com/yourusername/IterativeRegularization.jl/workflows/CI/badge.svg)](https://github.com/yourusername/IterativeRegularization.jl/actions)
[![Coverage](https://codecov.io/gh/yourusername/IterativeRegularization.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/yourusername/IterativeRegularization.jl)

A Julia package for solving two-point boundary value problems (TPBVPs) via kinodynamic motion planning.

> ⚠️ **Work in progress**
>
> This repository is actively being developed and the API, files, and examples may change without notice. Use at your own risk.


## Purpose
DynamicPlanning.jl will provide fast, flexible implementations of baseline and cutting-edge asymptotically optimal kinodynamic motion planners with coming support for temporal logic.

## Installation

```julia
using Pkg
Pkg.add("https://github.com/JuliaDifferentialGames/DynamicPlanning.jl.git")
```

<!--
## Development Status

**Currently Implemented:**
- ✅ Package structure

**In Progress:**
- 🚧 wip

**Planned:**
- 📋 todo

## Forthcoming Methods

-fmt...

## Features

### Problem Types
- `InverseProblem`: General nonlinear inverse problems
- `LinearInverseProblem`: Specialized type for linear problems
- Support for priors, constraints, and noise models

### Callbacks and Analysis
- `DiscrepancyPrinciple`: Morozov's discrepancy principle for stopping
- `ResidualCallback`: Monitor convergence
- `SolutionSaver`: Save intermediate solutions
- Regularization path computation and L-curve analysis

### Integration with Julia Ecosystem
- Automatic differentiation via ForwardDiff.jl and Zygote.jl (planned)
- Neural network regularizers via Lux.jl (planned)
- GPU acceleration support (planned)
- Statistical priors via Distributions.jl (planned)


## Theory and References

The package implements methods from:

1. Kaltenbacher, B., Neubauer, A., & Scherzer, O. (2008). *Iterative Regularization Methods for Nonlinear Ill-Posed Problems*. Walter de Gruyter.


## Contributing

Contributions are welcome! The package is structured to make adding new algorithms straightforward:

1. Define your algorithm struct inheriting from `AbstractIterativeRegularizationAlgorithm`
2. Implement a `_solve` method for your algorithm
3. Add tests and documentation
4. Submit a pull request

See `src/algorithms.jl` and the Nonlinear Landweber implementation in `src/solve.jl` as examples.

-->
## Citation

If you use this package in your research, please cite:

```bibtex
@software{dynamicplanning_jl,
  author = {Outland, Bennet},
  title = {DynamicPlanning.jl: Kinodynamic Motion Planning in Julia},
  year = {2025},
  url = {https://github.com/BennetOutland/DynamicPlanning.jl}
}
```

## License

MIT License - see LICENSE file for details.

## Acknowledgments

This package follows the design principles of the [SciML](https://sciml.ai/) ecosystem and draws inspiration from:
- MotionPlanning.jl
- DifferentialEquations.jl
- Optimization.jl

## Disclosure of Generative AI Usage

Generative AI, Claude, was used in the creation of this library as a programming aid including guided code generation, assistance with performance optimization, and for assistance in writing documentation. All code and documentation included in this repository, whether written by the author(s) or generative AI, has been reviewed by the author(s) for accuracy and has completed a verification and validation process upon release.
