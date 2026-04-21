# GCP VM 완전 복구 가이드 — Drone Bombard 환경

> **이 파일 하나만 보고 새 VM에서 전체 환경을 원상 복구할 수 있도록 작성됨.**
> 순서대로 실행하면 됨. VM IP: `136.113.193.83`
> 접속: **https://136.113.193.83** (HTTP는 자동으로 HTTPS로 리다이렉트)

---

## 전체 아키텍처

```
[브라우저]
    │ HTTP :80
    ▼
[nginx container]              ← Docker Compose
    │ proxy :8080
    ▼
[guacamole container]          ← Docker Compose
    │ guacd protocol :4822
    ▼
[guacd container]              ← Docker Compose
    │ VNC :5901  (host.docker.internal)
    ▼
[TigerVNC — VM 호스트]          ← 호스트 직접 설치
    │ X11 display
    ▼
[XFCE4 세션 on VM]

[Docker container: drone-bombard-harmonic]  ← Artifact Registry에서 pull
    │ ROS 2 Humble + PX4 SITL + Gazebo Harmonic + SAC(SB3)
    └─ /workspace/ros2_ws  (호스트 볼륨 마운트)
```

---

## Step 0: Ubuntu 초기화

### 0-1. 패키지 업데이트 + 필수 도구

```bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y \
  curl wget git vim htop tmux unzip \
  build-essential ca-certificates gnupg lsb-release \
  software-properties-common apt-transport-https \
  net-tools ufw
```

### 0-2. 타임존 설정

```bash
sudo timedatectl set-timezone Asia/Seoul
timedatectl   # 확인
```

### 0-3. UFW 방화벽 설정

```bash
sudo ufw allow 22/tcp     # SSH
sudo ufw allow 80/tcp     # HTTP (Guacamole)
sudo ufw allow 443/tcp    # HTTPS (선택)
# VNC 포트(5901)는 외부에 열지 않음 — Docker 내부에서만 접근
sudo ufw enable
sudo ufw status
```

### 0-4. 기본 사용자 디렉토리 구조

```bash
sudo mkdir -p /opt/drone-bombard
sudo chown $USER:$USER /opt/drone-bombard
```

---

## Step 1: NVIDIA Driver + CUDA Toolkit 설치

> GCP VM에 L4 GPU가 연결된 경우. GPU 없는 VM은 이 단계 건너뜀.

### 1-1. NVIDIA 드라이버 설치

```bash
# 기존 nouveau 비활성화
sudo bash -c 'echo "blacklist nouveau" > /etc/modprobe.d/blacklist-nouveau.conf'
sudo bash -c 'echo "options nouveau modeset=0" >> /etc/modprobe.d/blacklist-nouveau.conf'
sudo update-initramfs -u

# NVIDIA 드라이버 설치 (ubuntu-drivers 사용)
sudo apt-get install -y ubuntu-drivers-common
sudo ubuntu-drivers autoinstall

# 또는 버전 지정 설치 (L4 권장: 535 이상)
# sudo apt-get install -y nvidia-driver-535

sudo reboot   # 재부팅 필수
```

재부팅 후 확인:

```bash
nvidia-smi   # GPU 정보 출력되면 성공
```

### 1-2. CUDA Toolkit 설치

```bash
# CUDA 12.x keyring 등록
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update

# CUDA 12 설치 (런타임 + 툴킷)
sudo apt-get install -y cuda-toolkit-12-3

# 환경변수 등록
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

nvcc --version   # 확인
```

---

## Step 2: Docker + Docker Compose 설치

### 2-1. Docker 공식 저장소 등록

```bash
# GPG 키 등록
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 저장소 등록
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 2-2. sudo 없이 docker 사용 설정

```bash
sudo usermod -aG docker $USER
newgrp docker        # 현재 셸에 즉시 적용 (또는 로그아웃 후 재로그인)
docker ps            # 확인
```

### 2-3. NVIDIA Container Toolkit 설치

```bash
# 저장소 등록
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Docker 런타임에 NVIDIA 등록
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 확인
docker run --rm --gpus all nvidia/cuda:12.3.0-base-ubuntu22.04 nvidia-smi
```

### 2-4. Docker 자동 시작 설정

```bash
sudo systemctl enable docker
sudo systemctl status docker
```

---

## Step 3: SSH 키 생성 + GitHub 연동

### 3-1. SSH 키 생성

```bash
ssh-keygen -t ed25519 -C "nachoigpt@gmail.com" -f ~/.ssh/id_ed25519
# 패스프레이즈: 원하는 값 또는 엔터(빈 패스프레이즈)

# SSH 에이전트에 키 등록
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# 공개키 출력 → GitHub에 등록할 내용
cat ~/.ssh/id_ed25519.pub
```

### 3-2. GitHub에 공개키 등록

1. https://github.com/settings/keys 접속
2. **New SSH key** 클릭
3. Title: `GCP-VM-nachoigpt`, Key: 위 공개키 붙여넣기
4. Add SSH key

### 3-3. 연결 테스트

```bash
ssh -T git@github.com
# "Hi nachoigpt! You've successfully authenticated..." 출력되면 성공
```

### 3-4. Git 전역 설정

```bash
git config --global user.name "drone-bombard"
git config --global user.email "nachoigpt@gmail.com"
```

---

## Step 4: 프로젝트 Clone

```bash
cd /opt/drone-bombard
git clone git@github.com:<YOUR_ORG>/Drone-Bombard-Simulation.git
# 예: git clone git@github.com:nachoigpt/Drone-Bombard-Simulation.git

cd Drone-Bombard-Simulation
ls   # CLAUDE.md, notes/, ros2_ws/, gazebo_models/ 등 확인
```

---

## Step 5: Claude CLI + RTK 설치 + Claude Code 설정

### 5-1. jq 설치 (RTK hook 의존성)

```bash
sudo apt-get install -y jq
jq --version   # jq-1.6 이상 확인
```

### 5-2. Node.js + Claude Code CLI 설치

```bash
# npm 설치 (Node.js 필요)
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs

# Claude Code CLI 설치
npm install -g @anthropic/claude-code

# 확인
claude --version   # 2.1.x 확인

# 첫 실행 시 API 키 인증
claude
# → 브라우저 열림 → Anthropic 계정 로그인 → 완료
```

### 5-3. RTK (Rust Token Killer) 설치

RTK는 Claude Code 명령을 토큰 효율적으로 재작성하는 CLI 프록시 (60~90% 토큰 절약).

```bash
# RTK 바이너리 다운로드 (공식 설치)
# 방법 A — cargo (Rust 설치된 경우)
cargo install rtk
# 설치 후 ~/.cargo/bin/rtk 생성됨, PATH에 이미 포함되어 있으면 완료

# 방법 B — 바이너리 직접 배치
# GitHub Releases에서 linux-x86_64 바이너리 다운로드 후:
mkdir -p ~/.local/bin
# (다운로드한 바이너리를) cp rtk ~/.local/bin/rtk
chmod +x ~/.local/bin/rtk

# PATH 등록 확인 (~/.profile에 이미 있음)
echo $PATH | grep -o "[^:]*local/bin[^:]*"   # ~/.local/bin 포함 확인

# 확인
rtk --version   # rtk 0.36.0
rtk gain        # 절약 통계 (처음엔 0)
```

### 5-4. Claude Code hooks + 전역 설정

RTK hook 파일과 settings.json은 git repo에 없어서 수동 복원 필요.

```bash
# hook 디렉토리 생성
mkdir -p ~/.claude/hooks

# RTK PreToolUse hook 설치 (rtk가 설치된 경우 자동 생성 가능)
rtk install-hook   # 또는 수동으로 아래 내용으로 파일 생성

# ~/.claude/hooks/rtk-rewrite.sh 내용은 RTK 공식 문서 참조
# (claude code `rtk setup` 명령으로 자동 설치되기도 함)
```

`~/.claude/settings.json` 내용:

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "/home/nachoigpt/.claude/hooks/rtk-rewrite.sh"
          }
        ]
      }
    ]
  },
  "extraKnownMarketplaces": {
    "obsidian-skills": {
      "source": {
        "source": "github",
        "repo": "kepano/obsidian-skills"
      }
    }
  },
  "skipDangerousModePermissionPrompt": true
}
```

> **주의:** `command` 경로의 `nachoigpt`를 실제 사용자명으로 교체.

### 5-5. Claude Code 전역 CLAUDE.md 설정

```bash
# ~/.claude/CLAUDE.md — RTK.md 불러오기
cat > ~/.claude/CLAUDE.md << 'EOF'
@RTK.md
EOF

# ~/.claude/RTK.md — RTK 사용 지침
# RTK 공식 문서 참조 또는 기존 파일 복사
# (이 repo 내에는 포함 안 됨 — 개인 설정)
```

### 5-6. Claude Code obsidian-skills 플러그인 설치

```bash
# settings.json에 extraKnownMarketplaces 등록 후
claude
# Claude Code 내에서: /install obsidian-skills
# 또는 marketplace에서 직접 설치
```

---

## Step 6: Google Cloud (gcloud) + Artifact Registry 인증

### 6-1. gcloud CLI 설치

```bash
# gcloud 저장소 등록
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/cloud.google.gpg

echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] \
  https://packages.cloud.google.com/apt cloud-sdk main" \
  | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list

sudo apt-get update
sudo apt-get install -y google-cloud-cli

gcloud version   # 확인
```

### 6-2. GCP 인증 + 프로젝트 설정

```bash
# 서비스 계정 키 파일이 있는 경우
gcloud auth activate-service-account --key-file=/path/to/key.json

# 또는 사용자 계정으로 인증 (브라우저 필요)
gcloud auth login

# 프로젝트 설정
gcloud config set project charming-league-481306-d8
gcloud config set compute/zone us-central1-a
```

### 6-3. Artifact Registry Docker 인증 설정

```bash
gcloud auth configure-docker us-central1-docker.pkg.dev
# ~/.docker/config.json에 credHelper 등록됨

# 확인
cat ~/.docker/config.json | grep credHelpers
```

---

## Step 7: Docker 이미지 Pull + 컨테이너 실행

### 7-1. 이미지 Pull

```bash
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest
```

### 7-2. 컨테이너 최초 실행

```bash
xhost +local:docker

docker run -itd \
  --gpus all \
  --net=host \
  --privileged \
  --ipc=host \
  --name drone-bombard-harmonic \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  --log-driver=json-file \
  --log-opt max-size=10m \
  --log-opt max-file=3 \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest \
  /bin/bash

docker ps   # drone-bombard-harmonic 컨테이너 확인
```

### 7-3. 컨테이너 재접속 (VM 재부팅 후)

```bash
xhost +local:docker && docker start -ai drone-bombard-harmonic
```

---

## Step 8: ROS 2 / Gazebo 패키지 빌드

> 컨테이너 내부에서 실행

```bash
docker exec -it drone-bombard-harmonic bash

# 컨테이너 내부
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

cd /workspace/ros2_ws

# 전체 빌드
colcon build --symlink-install
source install/setup.bash

# 특정 패키지만 빌드
colcon build --packages-select rl_navigation
source install/setup.bash

# Gazebo 리소스 경로 설정
export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds

# 빌드 확인
ros2 pkg list | grep rl_navigation
```

> **⚠️ source 순서 필수:** `/root/ros2_ws/install/setup.bash` → `/workspace/ros2_ws/install/setup.bash`
> 순서 틀리면 `px4_msgs` import 에러로 에피소드 노드 silent crash.

---

## Step 9: TigerVNC + XFCE4 설치 (VM 호스트)

### 9-1. 패키지 설치

```bash
sudo apt-get install -y \
  tigervnc-standalone-server \
  xfce4 xfce4-goodies \
  dbus-x11 \
  xfonts-base

# 한글 폰트 설치 (없으면 GUI에서 한글 깨짐)
sudo apt-get install -y \
  fonts-nanum fonts-nanum-coding fonts-nanum-extra fonts-unfonts-core \
  locales

# 한글 locale 생성
sudo locale-gen ko_KR.UTF-8
sudo update-locale LANG=ko_KR.UTF-8

fc-cache -fv   # 폰트 캐시 갱신
```

### 9-2. VNC 비밀번호 설정

```bash
vncpasswd
# 비밀번호 입력 (최대 8자 유효, DES 기반)
# view-only 비밀번호: n (불필요)
```

### 9-3. xstartup 스크립트 설치

```bash
mkdir -p ~/.vnc
cp /opt/drone-bombard/Drone-Bombard-Simulation/notes/Environment/vnc-xstartup ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup
cat ~/.vnc/xstartup   # 내용 확인
```

xstartup 내용 (참고):

```bash
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u)
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# 한글 표시를 위한 locale 설정
export LANG=ko_KR.UTF-8
export LC_ALL=ko_KR.UTF-8
export LANGUAGE=ko_KR:ko

exec dbus-launch --exit-with-session startxfce4
```

### 9-4. VNC 수동 테스트

```bash
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no -SecurityTypes VncAuth
vncserver -list   # :1 출력 확인
```

### 9-5. systemd 자동 시작 등록

```bash
sudo cp /opt/drone-bombard/Drone-Bombard-Simulation/notes/Environment/vncserver.service \
     /etc/systemd/system/vncserver@.service
sudo systemctl daemon-reload
sudo systemctl enable vncserver@1
sudo systemctl start  vncserver@1
sudo systemctl status vncserver@1
```

### 9-6. xfce4-screensaver 비활성화

> VNC 세션 안에서 잠금 화면이 뜨지 않도록 설정 (Linux 계정 비밀번호 프롬프트 방지)

```bash
# VNC 세션이 실행 중인 상태에서 DISPLAY 지정하여 실행
DISPLAY=:1 xfconf-query -c xfce4-screensaver -p /lock/enabled \
  -n -t bool -s false
DISPLAY=:1 xfconf-query -c xfce4-screensaver -p /saver/enabled \
  -n -t bool -s false
DISPLAY=:1 xfconf-query -c xfce4-screensaver -p /saver/idle-activation/enabled \
  -n -t bool -s false

# 설정 확인
DISPLAY=:1 xfconf-query -c xfce4-screensaver -p /lock/enabled
DISPLAY=:1 xfconf-query -c xfce4-screensaver -p /saver/enabled

# xfce4-screensaver 완전 제거 (권장 — 잠금 기능 자체를 없앰)
sudo apt-get remove -y xfce4-screensaver
```

---

## Step 10: Guacamole Docker Compose 스택 구성

### 10-1. 작업 디렉토리 확인

```bash
ls /opt/drone-bombard/guacamole-stack/
# docker-compose.yml  nginx.conf  initdb.sql  (이미 존재해야 함)
# 없으면: git clone으로 복구 (notes/Environment/에 백업됨)
```

백업에서 복원:

```bash
cp /opt/drone-bombard/Drone-Bombard-Simulation/notes/Environment/docker-compose.yml \
   /opt/drone-bombard/guacamole-stack/
cp /opt/drone-bombard/Drone-Bombard-Simulation/notes/Environment/nginx.conf \
   /opt/drone-bombard/guacamole-stack/
cp /opt/drone-bombard/Drone-Bombard-Simulation/notes/Environment/initdb.sql \
   /opt/drone-bombard/guacamole-stack/
```

### 10-2. SSL 인증서 생성 (자체 서명, 10년)

> 도메인 없이 IP만으로 HTTPS 구성. 브라우저에서 "연결이 안전하지 않음" 경고 1회 무시 후 접속 가능.

```bash
mkdir -p /opt/drone-bombard/guacamole-stack/ssl

openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
  -keyout /opt/drone-bombard/guacamole-stack/ssl/nginx.key \
  -out    /opt/drone-bombard/guacamole-stack/ssl/nginx.crt \
  -subj "/C=KR/ST=Seoul/L=Seoul/O=DroneBombard/OU=Research/CN=136.113.193.83" \
  -addext "subjectAltName=IP:136.113.193.83"
```

### 10-3. initdb.sql 생성 (최초 1회 또는 재생성 필요 시)

```bash
docker run --rm guacamole/guacamole:1.5.5 \
  /opt/guacamole/bin/initdb.sh --postgresql \
  > /opt/drone-bombard/guacamole-stack/initdb.sql
```

### 10-3. 스택 기동

```bash
cd /opt/drone-bombard/guacamole-stack
docker compose up -d

# 상태 확인 (postgres 헬스체크 통과 후 guacamole 기동 — 약 30~60초)
docker compose ps
docker compose logs -f guacamole   # "Guacamole is now listening on port 8080" 확인
```

### 10-4. 접속 확인

```bash
curl -I http://136.113.193.83/guacamole/
# HTTP/1.1 200 OK 또는 302 반환 확인
```

---

## Step 11: Guacamole UI에서 VNC 연결 등록

1. http://136.113.193.83 접속
2. `guacadmin` / `guacadmin` 로그인 **(첫 로그인 후 즉시 비밀번호 변경)**
3. 우상단 사용자 메뉴 → **Settings** → **Connections** → **New Connection**

| 항목 | 값 |
|------|-----|
| Name | `VM-XFCE4` |
| Protocol | `VNC` |
| Hostname | `host.docker.internal` |
| Port | `5901` |
| Password | vncpasswd에서 설정한 값 (8자 한도) |

4. **Save** → Home 화면에서 `VM-XFCE4` 클릭 → XFCE4 데스크탑 표시 확인

---

## Step 12: Obsidian AppImage 설치 + Vault 설정

> XFCE4 GUI 환경에서 연구 노트를 관리하는 로컬 Obsidian 설치.
> VNC → Guacamole로 접속 후 XFCE4 데스크탑에서 Obsidian GUI 사용.

### 12-1. AppImage 다운로드

```bash
mkdir -p ~/Applications

# Obsidian 1.x AppImage 다운로드 (GitHub Releases에서 최신 버전 확인)
# https://github.com/obsidianmd/obsidian-releases/releases
# 현재 설치 버전: 1.12.7
wget -O ~/Applications/Obsidian.AppImage \
  "https://github.com/obsidianmd/obsidian-releases/releases/download/v1.12.7/Obsidian-1.12.7.AppImage"

chmod +x ~/Applications/Obsidian.AppImage
```

### 12-2. 첫 실행 (vault 등록)

```bash
# XFCE4 VNC 세션에서 터미널 열고 실행
DISPLAY=:1 ~/Applications/Obsidian.AppImage --no-sandbox &

# 또는 직접 파일 더블클릭 (XFCE4 데스크탑)
```

첫 실행 시 Vault 설정:
1. **Open folder as vault** 선택
2. 경로: `/opt/drone-bombard/Drone-Bombard-Simulation/notes`
3. Vault 이름: `drone-bombard-research`

### 12-3. Community Plugin 설치 (Dataview)

1. **Settings** → **Community Plugins** → "Turn on community plugins"
2. **Browse** → `Dataview` 검색 → Install → Enable
3. Dataview v0.5.70 (또는 최신)

### 12-4. 외관 설정

1. **Settings** → **Appearance**
   - Font: **Nanum Gothic** (한글 렌더링)
   - Interface font: Nanum Gothic

2. **Graph View** 색상 그룹 설정 (`notes/.obsidian/graph.json`에 이미 저장됨 — 자동 적용):
   - `experiments/` → 초록
   - `errors/` → 빨강
   - `research/` → 파랑
   - `sessions/` → 보라

### 12-5. XFCE4 자동시작 등록

```bash
mkdir -p ~/.config/autostart

cat > ~/.config/autostart/obsidian.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=Obsidian
Exec=/home/nachoigpt/Applications/Obsidian.AppImage --no-sandbox
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
EOF
```

> **주의:** `nachoigpt`를 실제 사용자명으로 교체.

### 12-6. 데스크탑 아이콘 등록 (선택)

```bash
cp ~/.config/autostart/obsidian.desktop ~/Desktop/
chmod +x ~/Desktop/obsidian.desktop
```

---

## Step 13: 검증 체크리스트

```bash
# 1. NVIDIA GPU 확인
nvidia-smi

# 2. Docker 확인
docker ps | grep drone-bombard-harmonic   # 컨테이너 실행 중

# 3. VNC 확인
vncserver -list              # :1 표시
ss -tlnp | grep 5901         # LISTEN 확인

# 4. Guacamole 스택 확인
cd /opt/drone-bombard/guacamole-stack
docker compose ps            # 4개 서비스 모두 Up (healthy)

# 5. HTTP 응답 확인
curl -sI http://136.113.193.83/guacamole/ | head -1
# → HTTP/1.1 200 OK 또는 302 Found

# 6. ROS 2 빌드 확인 (컨테이너 내부)
docker exec drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   ros2 pkg list | grep rl_navigation"

# 7. Claude Code + RTK 확인
claude --version             # 2.1.x
rtk --version                # rtk 0.36.0
rtk gain                     # 통계 출력 (에러 없으면 정상)
cat ~/.claude/settings.json | grep rtk-rewrite   # hook 등록 확인

# 8. Obsidian 실행 확인 (VNC 세션에서)
DISPLAY=:1 ~/Applications/Obsidian.AppImage --no-sandbox &
# → XFCE4에 Obsidian 창 뜨면 성공
# → drone-bombard-research vault 자동 열림 확인
```

---

## 일상 운영 명령어

```bash
# Guacamole 스택 재시작
cd /opt/drone-bombard/guacamole-stack && docker compose restart

# VNC 상태
sudo systemctl status vncserver@1

# RL 학습 시작 (Fresh)
docker exec -d drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds && \
   cd /workspace/ros2_ws && ros2 run rl_navigation train_sac \
   > /tmp/production_train.log 2>&1"

# 학습 로그 모니터
docker exec drone-bombard-harmonic tail -f /tmp/production_train.log
```

---

## VM 선점(Preemption) 후 복구

GCP Spot VM 선점 후 재시작 시:

```bash
# 1. Docker 컨테이너 재시작 (데이터 보존)
xhost +local:docker && docker start drone-bombard-harmonic

# 2. Guacamole 스택 재시작
cd /opt/drone-bombard/guacamole-stack && docker compose up -d

# 3. VNC는 systemd가 자동 재시작 (확인만)
sudo systemctl status vncserver@1

# 4. 학습 재개 (체크포인트에서)
docker exec -d drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds && \
   cd /workspace/ros2_ws && ros2 run rl_navigation train_sac \
     --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip \
   > /tmp/production_train.log 2>&1"
```

---

## 완전 초기화 (스택 재설치)

```bash
# Guacamole 스택만 초기화 (postgres 데이터 포함)
cd /opt/drone-bombard/guacamole-stack
docker compose down -v        # volume까지 삭제
docker compose up -d          # initdb.sql 재적용하며 재생성
```

---

## 주요 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `UnknownHostException: nachoigpt` | guacd 컨테이너가 호스트 hostname 미해석 | `extra_hosts: nachoigpt:host-gateway`를 **guacd 서비스에도** 추가 |
| `AuthFailureException TLSVnc(258)` | guacd가 TLSVnc 시도, TigerVNC 거부 | vncserver 실행 시 `-SecurityTypes VncAuth` 추가 |
| XFCE4 화면 안 뜸 (Framebuffer 0) | dbus 세션 없음 | xstartup에 `exec dbus-launch --exit-with-session startxfce4` |
| 잠금 화면 비밀번호 요구 | xfce4-screensaver가 Linux 계정 비밀번호 요구 | xfconf-query 비활성화 또는 `apt remove xfce4-screensaver` |
| VNC 비밀번호 불일치 (10초 끊김) | VNC DES 8자 한도 초과 | vncpasswd는 8자까지만 유효, Guacamole DB에 동일 값 등록 확인 |
| `px4_msgs` import 에러 (silent crash) | source 순서 오류 | `/root/ros2_ws` → `/workspace/ros2_ws` 순서 필수 |
| `docker: permission denied` | docker 그룹 미등록 | `sudo usermod -aG docker $USER && newgrp docker` |
| RTK hook `[rtk] WARNING: jq is not installed` | jq 미설치 | `sudo apt-get install -y jq` |
| Obsidian AppImage 실행 안 됨 | FUSE 미설치 | `sudo apt-get install -y libfuse2` 또는 `--appimage-extract-and-run` 옵션 사용 |

---

## 주요 계정 정보

| 서비스 | ID | PW / 비고 |
|--------|----|----|
| Guacamole 관리자 | `guacadmin` | `guacadmin` (첫 로그인 후 변경) |
| PostgreSQL | `guacamole_user` | `guac_pg_pass_2026` |
| VNC | — | vncpasswd로 설정 (8자 한도) |
| GCP 프로젝트 | — | `charming-league-481306-d8` |
| Artifact Registry | — | `us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/` |

---

## 파일 구조 (이 폴더)

| 파일 | 설명 |
|------|------|
| `README.md` | 이 가이드 |
| `docker-compose.yml` | Guacamole 전체 스택 정의 |
| `nginx.conf` | WebSocket 리버스 프록시 설정 |
| `initdb.sql` | PostgreSQL 스키마 (guacamole 공식) |
| `vncserver.service` | TigerVNC systemd 유닛 |
| `vnc-xstartup` | VNC 세션 xstartup 스크립트 |
