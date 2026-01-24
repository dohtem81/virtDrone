# virtDrone

A drone simulator project with three main components: simulator (physics), drone (model and control), and web-based control interface with visualization.

## Technologies

- **Simulator and Drone**: Implemented in C++ for performance.
- **Web Interface**: Backend in Python using FastAPI, with potential frontend in JavaScript/React.

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

1. Clone the repository.
2. Ensure Docker and Docker Compose are installed.
3. Run `docker-compose up --build` to start all components in containers.
4. Access the web interface at http://localhost:8000.
5. For local development: Follow C++ build instructions in `tools/build/`, install Python deps with `pip install -r requirements.txt`, and run components individually.
6. Refer to `docs/setup/` for detailed build instructions.

## License

See [LICENSE](LICENSE) for details.