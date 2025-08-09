# 2D Physics Engine Simulation

This repository contains a 2D physics engine implemented in C++. The engine simulates the motion of multiple bodies under the influence of gravitational force and includes rudimentary collision handling.

## Features

- **Vector Class**: Represents 2D vectors with comprehensive operations, including normalization and dot products.
- **Body Class**: Models particles with mass, radius, position, velocity, and accumulated force.
- **N-Body Gravitational Interaction**: Calculates pairwise gravitational forces among any number of bodies.
- **Elastic Collisions**: Resolves simple collisions using an impulse-based response.
- **Energy Calculations**: Computes total kinetic and potential energy for diagnostic purposes.
- **Simulation Loop**: Performs time-step integration for all bodies in the system.

## Getting Started

### Prerequisites

- C++ compiler (e.g., GCC, Clang, or MSVC)

### Building

1. Clone the repository: git clone https://github.com/dsoon0400/2D-Physics-Engine-Sim.git

2. Navigate to the project directory: cd 2d-physics-engine

3. Compile the source code: `g++ -std=c++17 -o engine engine.cpp`

4. Run the compiled program: `./engine`

### Usage

The program will simulate the motion of multiple bodies under gravitational force and print their final positions and energy values after the simulation. You can modify the initial conditions (masses, positions, velocities, and radii) in the `main` function to observe different scenarios.

## Contributing

Contributions are welcome! If you have any suggestions, improvements, or additional features to add, please open an issue or submit a pull request.
