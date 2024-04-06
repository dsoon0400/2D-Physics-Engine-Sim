# 2D Physics Engine Simulation

This repository contains a basic 2D physics engine implemented in C++. The engine simulates the motion of two bodies under the influence of gravitational force, using concepts from classical mechanics and numerical methods.

## Features

- **Vector Class**: Represents 2D vectors with operations like addition, subtraction, and scalar multiplication.
- **Body Class**: Represents a particle in the simulation with properties like mass, position, and velocity.
- **Gravitational Force Calculation**: Computes the gravitational force between two bodies using Newton's law of gravitation.
- **Position Update**: Updates the positions of bodies based on their velocities and the elapsed time.
- **Simulation Loop**: Performs the simulation over a specified number of time steps (e.g., 365 days).

## Getting Started

### Prerequisites

- C++ compiler (e.g., GCC, Clang, or MSVC)

### Building

1. Clone the repository: git clone [https://github.com/dsoon0400/2d-physics-engine.git](https://github.com/dsoon0400/2D-Physics-Engine-Sim/edit/main/README.md)

2. Navigate to the project directory: cd 2d-physics-engine

3. Compile the source code: g++ -o physics_engine main.cpp

4. Run the compiled program: ./physics_engine

### Usage

The program will simulate the motion of two bodies under gravitational force and print their final positions after the simulation. You can modify the initial conditions (masses, positions, and velocities) in the `main` function to observe different scenarios.

## Contributing

Contributions are welcome! If you have any suggestions, improvements, or additional features to add, please open an issue or submit a pull request.
