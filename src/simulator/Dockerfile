# Dockerfile for simulator component (C++)
FROM ubuntu:20.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy source
COPY CMakeLists.txt .
COPY src/ ./src/
COPY libs/ ./libs/

# Build the simulator
RUN mkdir build && cd build && cmake .. && make

# Expose port if needed (e.g., for integration)
EXPOSE 8080

# Run the simulator (placeholder, adjust as needed)
CMD ["./build/simulator_executable"]  # Replace with actual executable