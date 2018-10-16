Library ▸ Astrodynamics
=======================

Orbit, attitude, access.

[![Build Status](https://travis-ci.com/open-space-collective/library-astrodynamics.svg?branch=master)](https://travis-ci.com/open-space-collective/library-astrodynamics)
[![Code Coverage](https://codecov.io/gh/open-space-collective/library-astrodynamics/branch/master/graph/badge.svg)](https://codecov.io/gh/open-space-collective/library-astrodynamics)
[![Documentation](https://img.shields.io/readthedocs/pip/stable.svg)](https://open-space-collective.github.io/library-astrodynamics)
[![GitHub version](https://badge.fury.io/gh/open-space-collective%2Flibrary-astrodynamics.svg)](https://badge.fury.io/gh/open-space-collective%2Flibrary-astrodynamics)
[![PyPI version](https://badge.fury.io/py/LibraryAstrodynamicsPy.svg)](https://badge.fury.io/py/LibraryAstrodynamicsPy)

<img src="./docs/assets/example.svg" height="500px" width="auto">

## Warning

Library **name** and **license** are yet to be defined.

Please check the following projects:

- [Naming Project](https://github.com/orgs/open-space-collective/projects/1)
- [Licensing Project](https://github.com/orgs/open-space-collective/projects/2)

*⚠ This library is still under heavy development. Do not use!*

## Structure

The **Astrodynamics** library exhibits the following structure:

```txt
├── Trajectory
│   ├── State
│   ├── Orbit
│   │   ├── Models
│   │   │   ├── Kepler
│   │   │   │   └── Classical Orbital Elements (COE)
│   │   │   ├── SGP4
│   │   │   │   └── Two-Line Element set (TLE)
│   │   │   └── Propagator
│   │   ├── Pass
│   │   └── Utilities
│   └── Composite
├── Flight
│   └── Profile
│       └── State
├── Access
└── State Profile
```

## Documentation

The documentation can be found here:

- [C++](https://open-space-collective.github.io/library-astrodynamics)
- [Python](./bindings/python/docs)

## Tutorials

Various tutorials are available here:

- [C++](./tutorials/cpp)
- [Python](./tutorials/python)

## Setup

### Development

Using [Docker](https://www.docker.com) is recommended, as the development tools and dependencies setup is described in the provided [Dockerfile](./tools/development/docker/Dockerfile).

Instructions to install Docker can be found [here](https://docs.docker.com/install/).

Start the development environment:

```bash
./tools/development/start.sh
```

This will also build the `openspacecollective/library-astrodynamics:latest` Docker image, if not present already.

If installing Docker is not an option, please manually install the development tools (GCC, CMake) and the dependencies.
The procedure should be similar to the one described in the [Dockerfile](./tools/development/docker/Dockerfile).

### Build

From the development environment:

```bash
./build.sh
```

Manually:

```bash
mkdir -p build
cd build
cmake ..
make
```

### Test

From the development environment:

```bash
./test.sh
```

Manually:

```bash
./bin/library-astrodynamics.test
```

## Dependencies

The **Astrodynamics** library internally uses the following dependencies:

| Name        | Version | License                | Link                                                                                                                 |
|-------------|---------|------------------------|----------------------------------------------------------------------------------------------------------------------|
| Boost       | 1.67.0  | Boost Software License | [boost.org](https://www.boost.org)                                                                                   |
| Eigen       | 3.3.4   | MPL2                   | [eigen.tuxfamily.org](http://eigen.tuxfamily.org/index.php)                                                          |
| SGP4        | master  | Apache-2.0             | [github.com/dnwrnr/sgp4](https://github.com/dnwrnr/sgp4)                                                             |
| NLopt       | master  | LGPL                   | [github.com/stevengj/nlopt](https://github.com/stevengj/nlopt)                                                       |
| Core        | master  | TBD                    | [github.com/open-space-collective/library-core](https://github.com/open-space-collective/library-core)               |
| Mathematics | master  | TBD                    | [github.com/open-space-collective/library-mathematics](https://github.com/open-space-collective/library-mathematics) |
| Physics     | master  | TBD                    | [github.com/open-space-collective/library-physics](https://github.com/open-space-collective/library-physics)         |

## Contribution

Contributions are more than welcome!

Please read our [contributing guide](CONTRIBUTING.md) to learn about our development process, how to propose fixes and improvements, and how to build and test the code.

## Special Thanks

*To be completed...*

## License

*To be defined...*