# ============================================================================
# YOLOs-CPP Docker Image
# ============================================================================
# Multi-stage build for optimized image size and caching
#
# Build: docker build -t yolos-cpp .
# Run:   docker run --rm -it yolos-cpp
# GPU:   docker run --gpus all --rm -it yolos-cpp
# ============================================================================

# -----------------------------------------------------------------------------
# Stage 1: Builder
# -----------------------------------------------------------------------------
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04 AS builder

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    ca-certificates \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /build

# Download and extract ONNX Runtime
ARG ONNXRUNTIME_VERSION=1.20.1
RUN wget -q https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz \
    && tar -xzf onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz \
    && mv onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION} /opt/onnxruntime \
    && rm onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz

# Copy source code
COPY . /build/src

# Build the project
WORKDIR /build/src
RUN mkdir -p build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DONNXRUNTIME_DIR=/opt/onnxruntime \
    && make -j$(nproc)

# -----------------------------------------------------------------------------
# Stage 2: Runtime
# -----------------------------------------------------------------------------
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04 AS runtime

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies only
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-core4.5d \
    libopencv-imgproc4.5d \
    libopencv-imgcodecs4.5d \
    libopencv-videoio4.5d \
    libopencv-highgui4.5d \
    libgtk-3-0 \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# Copy ONNX Runtime libraries
COPY --from=builder /opt/onnxruntime/lib /opt/onnxruntime/lib

# Copy built binaries
COPY --from=builder /build/src/build/image_inference /app/
COPY --from=builder /build/src/build/video_inference /app/
COPY --from=builder /build/src/build/camera_inference /app/
COPY --from=builder /build/src/build/class_image_inference /app/
COPY --from=builder /build/src/build/batch_image_inference /app/

# Copy models and data
COPY --from=builder /build/src/models /app/models
COPY --from=builder /build/src/data /app/data

# Set library path
ENV LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH

# Set working directory
WORKDIR /app

# Default inference target
ENV INFERENCE_TARGET=image_inference
ENV MODEL_PATH=models/yolo11n.onnx
ENV INPUT_PATH=data/dog.jpg
ENV LABELS_PATH=models/coco.names

# Healthcheck
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD test -f /app/${INFERENCE_TARGET} || exit 1

# Default command
CMD ./${INFERENCE_TARGET} ${MODEL_PATH} ${INPUT_PATH} ${LABELS_PATH}

# -----------------------------------------------------------------------------
# Labels
# -----------------------------------------------------------------------------
LABEL maintainer="YOLOs-CPP Team <https://github.com/Geekgineer/YOLOs-CPP>"
LABEL version="2.0.0"
LABEL description="High-Performance C++ Inference for YOLO Models"
LABEL org.opencontainers.image.source="https://github.com/Geekgineer/YOLOs-CPP"
