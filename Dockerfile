# fog-sw BUILDER
FROM ros:foxy-ros-base as fog-sw-builder

ARG UID=1000
ARG GID=1000
ARG BUILD_NUMBER
ARG COMMIT_ID
ARG GIT_VER

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    python3-bloom \
    fakeroot \
    dh-make \
    libboost-dev \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g $GID builder && \
    useradd -m -u $UID -g $GID -g builder builder && \
    usermod -aG sudo builder && \
    echo 'builder ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN mkdir -p /odometry2/packaging

COPY packaging/rosdep.yaml packaging/rosdep.sh /odometry2/packaging/
COPY underlay.repos /odometry2/

RUN /odometry2/packaging/rosdep.sh /odometry2

RUN chown -R builder:builder /odometry2

USER builder

VOLUME /odometry2/sources
WORKDIR /odometry2/sources

RUN rosdep update
