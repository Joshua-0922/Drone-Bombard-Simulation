ARG ISAACSIM_VERSION=5.1.0
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y
ENV ISAACLAB_PATH=/workspace/isaaclab

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential ncurses-term \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab v2.3.2 (pinned) — symlink so isaaclab.sh finds /isaac-sim
RUN git clone --depth 1 -b v2.3.2 https://github.com/isaac-sim/IsaacLab.git ${ISAACLAB_PATH}
WORKDIR ${ISAACLAB_PATH}
RUN ln -sf /isaac-sim _isaac_sim
RUN ./isaaclab.sh --install rsl_rl

# Extra deps via Isaac Sim's bundled Python 3.11
COPY drone_drop_system/docker/requirements.txt /tmp/requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements.txt

COPY drone_drop_system/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /workspace/isaaclab
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
