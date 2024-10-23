# Kalman-Filter-Scratch

Sandbox to implement Kalman Filter Projects from scratch in C++/Python.

## Getting Started

### Prerequisites

Before running the project, install the following header-only libraries:

- **Eigen**: For matrix operations.
- **matplotlibcpp**: For plotting graphs.

Since both libraries are header-only, no additional building is required. Ensure that the linker can locate them after installation.

### Running the 1D Kalman Filter

1. Clone the repository.
2. Navigate to the 1D Kalman Filter directory and create a build directory:
   ```bash
   cd <path-to-repository>/Kalman-Filter-Scratch/1D/
   mkdir build
   cd build
   cmake ..
   make
   ./Kalman1D
