FROM ubuntu:24.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and git
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    ninja-build \
    libgtest-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Build and install GTest
RUN cd /usr/src/gtest && cmake . -GNinja && ninja && cp lib/*.a /usr/lib/

# Copy source and build
COPY . .
RUN mkdir build && cd build && cmake .. -GNinja && ninja

# Expose port
EXPOSE 8081

# Default command
CMD ["./build/drone_executable"]  # Adjust to your executable
