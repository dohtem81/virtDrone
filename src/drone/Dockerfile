# Dockerfile for drone component (C++)
FROM ubuntu:20.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy source
COPY CMakeLists.txt .
COPY src/ ./src/
COPY libs/ ./libs/
COPY include/ ./include/

# Build the drone
RUN mkdir build && cd build && cmake .. -GNinja && ninja

# Expose port if needed
EXPOSE 8081

# Run the drone (placeholder)
CMD ["./build/drone_executable"]  # Replace with actual executable
