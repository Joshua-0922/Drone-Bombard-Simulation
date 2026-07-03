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

# NOTE (2026-07-03, verified live on isaac-sim:5.1.0): two pre-existing image
# quirks break the stock install; neither is caused by our code.
#   (1) isaac-sim:5.1.0 ships MANY `packaging/_structures.py` as DANGLING
#       symlinks across its bundled python archives (pip's vendored copy,
#       torch's vendored copy in omni.isaac.ml_archive, setuptools/_vendor,
#       the pip_prebundle dirs, ...). Any of them being dead makes pip AND
#       `import torch`/isaac core extensions die with
#       "No module named '..._structures'". Repair ALL of them by copying the
#       one real _structures.py over every dead link.
#   (2) `isaaclab.sh --install` then still fails to install the CORE `isaaclab`
#       package (sub-packages install; core build dies with "No module named
#       'pkg_resources'"). Ensure setuptools is present, and explicitly
#       editable-install the core package afterwards.
RUN GOOD=/isaac-sim/kit/python/lib/python3.11/site-packages/packaging/_structures.py; \
    find /isaac-sim -path '*/packaging/version.py' 2>/dev/null | while read v; do \
      d=$(dirname "$v"); t="$d/_structures.py"; \
      if [ ! -e "$t" ]; then rm -f "$t"; cp "$GOOD" "$t"; fi; \
    done
RUN /isaac-sim/python.sh -m pip install --no-cache-dir "setuptools>=66" wheel
RUN ./isaaclab.sh --install rsl_rl
# Belt-and-suspenders: ensure the core isaaclab package actually landed.
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation -e ${ISAACLAB_PATH}/source/isaaclab \
    && /isaac-sim/python.sh -c "import isaaclab; print('isaaclab', isaaclab.__version__)"

# Extra deps via Isaac Sim's bundled Python 3.11
COPY drone_drop_system/docker/requirements.txt /tmp/requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements.txt

COPY drone_drop_system/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /workspace/isaaclab
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
