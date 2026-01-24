# virtDrone

A drone simulator project with three main components: simulator (physics), drone (model and control), and web-based control interface with visualization.

## Technologies

- **Simulator and Drone**: Implemented in C++ for performance.
- **Web Interface**: Backend in Python using FastAPI, with potential frontend in JavaScript/React.
- **Testing**: Catch2 for C++ unit tests, pytest for Python tests.
- **Build System**: CMake with Ninja for C++, Docker for containerization.

## Components

- **Simulator**: Handles physics simulation including position, speed, tilt, etc. (C++).
- **Drone**: Manages motor speeds, temperatures, and control algorithms (C++).
- **Web Interface**: Provides control interface and visualization via web technologies (Python/FastAPI).

## Folder Structure

- `src/`: Main source code
  - `simulator/`: Physics simulation (C++)
  - `drone/`: Drone logic (C++)
  - `web/`: Web interface (Python)
- `libs/`: Shared libraries (C++)
- `docs/`: Documentation
- `tools/`: Build and utility scripts (includes CMake for C++)
- `tests/`: Unit and integration tests
- `examples/`: Sample projects
- `configs/`: Configuration files

## Getting Started

### Prerequisites

- Docker and Docker Compose
- Git

### Building the Project

The project uses Docker containers for consistent builds across environments.

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd virtDrone
   ```

2. Build the development environment:
   ```bash
   docker compose build dev
   ```

3. Build the project:
   ```bash
   docker compose run --rm build
   ```

   This will:
   - Configure the project with CMake using Ninja
   - Compile all C++ components
   - Build Python components

### Running Tests

The project uses Catch2 for C++ unit tests and pytest for Python tests.

1. Run C++ tests:
   ```bash
   docker compose run --rm dev ctest
   ```

2. Run Python tests:
   ```bash
   docker compose run --rm dev python -m pytest tests/unit/web/
   ```

### Running the Application

1. Start all components:
   ```bash
   docker compose up --build
   ```

2. Access the web interface at http://localhost:8000

### Local Development

For local development without Docker:

1. Install dependencies:
   - C++: CMake, Ninja, GCC/Clang
   - Python: `pip install -r requirements.txt`

2. Build C++ components:
   ```bash
   mkdir build && cd build
   cmake .. -GNinja
   ninja
   ```

3. Run tests:
   ```bash
   ctest  # For C++ tests
   python -m pytest tests/unit/web/  # For Python tests
   ```

4. Run components individually (refer to `docs/` for details).

## License

See [LICENSE](LICENSE) for details.