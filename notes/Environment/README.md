# Guacamole Stack — 설치 및 운영 가이드

접속: **http://136.113.193.83** → Guacamole 로그인 → XFCE4 데스크탑

---

## 아키텍처

```
브라우저 :80
  → nginx (Docker)
    → guacamole:8080 (Docker)
      → guacd:4822 (Docker)
        → VM 호스트 TigerVNC :5901
          → XFCE4 세션
```

---

## 최초 설치 순서

### Step 1: VM 호스트에 TigerVNC + XFCE4 설치

```bash
sudo apt-get update
sudo apt-get install -y tigervnc-standalone-server xfce4 xfce4-goodies dbus-x11

# VNC 비밀번호 설정 (나중에 Guacamole 연결 등록 시 입력)
vncpasswd

# xstartup 스크립트 설치
mkdir -p ~/.vnc
cp vnc-xstartup ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup

# 수동 테스트 실행
vncserver :1 -geometry 1920x1080 -depth 24
vncserver -list   # :1 표시 확인
```

### Step 2: VNC 자동 시작 (systemd)

```bash
sudo cp vncserver.service /etc/systemd/system/vncserver@.service
sudo systemctl daemon-reload
sudo systemctl enable vncserver@1
sudo systemctl start  vncserver@1
sudo systemctl status vncserver@1
```

### Step 3: Docker Compose 스택 기동

```bash
cd /opt/drone-bombard/guacamole-stack

# postgres 초기화 (최초 1회만 DB 생성됨)
docker compose up -d

# 상태 확인
docker compose ps
docker compose logs -f guacamole   # Guacamole 준비까지 30~60초 대기
```

### Step 4: Guacamole VNC 연결 등록

1. http://136.113.193.83 접속
2. `guacadmin` / `guacadmin` 로그인 **(첫 로그인 후 비밀번호 변경 권장)**
3. 우상단 메뉴 → Settings → Connections → New Connection
   - Name: `VM-XFCE4`
   - Protocol: `VNC`
   - Hostname: `host.docker.internal`
   - Port: `5901`
   - Password: (vncpasswd에서 설정한 값)
4. Save → Home 화면에서 VM-XFCE4 클릭 → XFCE4 데스크탑 표시

---

## 일상 운영

```bash
# 스택 재시작
docker compose restart

# 전체 중단
docker compose down

# 전체 재기동 (데이터 보존)
docker compose up -d

# 로그 확인
docker compose logs -f
docker compose logs guacamole

# VNC 상태
vncserver -list
sudo systemctl status vncserver@1
```

---

## VM 재부팅/선점 후 복구

```bash
# VNC 서버는 systemd가 자동 재시작
# Docker 스택만 재기동
cd /opt/drone-bombard/guacamole-stack
docker compose up -d
```

---

## 완전 초기화 (데이터 삭제 후 재설치)

```bash
docker compose down -v   # volume(postgres 데이터)까지 삭제
docker compose up -d     # 재생성 (initdb.sql 재적용)
```

---

## 파일 구조

| 파일 | 설명 |
|------|------|
| `docker-compose.yml` | 전체 스택 정의 (guacd, guacamole, postgres, nginx) |
| `nginx.conf` | WebSocket 프록시 설정 |
| `initdb.sql` | PostgreSQL 스키마 (guacamole/guacamole:1.5.5 공식) |
| `vncserver.service` | systemd 유닛 파일 |
| `vnc-xstartup` | VNC 세션 xstartup 스크립트 |

---

## 기본 계정

| 서비스 | ID | PW |
|--------|----|----|
| Guacamole 관리자 | `guacadmin` | `guacadmin` (첫 로그인 후 변경) |
| PostgreSQL | `guacamole_user` | `guac_pg_pass_2026` |
