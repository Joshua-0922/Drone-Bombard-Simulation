---
date: 2026-07-23
tags: [environment, vm, gcloud, access, ssh, stockout, guide, onboarding]
type: reference
status: active
owner: junsang
---

# L4 VM 접속 가이드 (Isaac 학습용) — 팀원용

> **목적:** Isaac 학습을 돌리는 GPU VM에 **접속이 안 될 때** 어떻게 뚫는지 실전 정리.
> 특히 **STOCKOUT(재고 없음)일 때 "될 때까지 재시도"** 방법까지.
> 전제: 로컬에 `gcloud` 설치 + 프로젝트 권한 로그인(`gcloud auth login`, `gcloud config set project ...`) 완료.

---

## 0. VM 기본 정보

| 항목 | 값 |
|------|-----|
| 인스턴스 이름 | `g2-standard-16-nvidia-l4-dev` |
| 존(zone) | `us-central1-c` |
| 프로젝트 | `charming-league-481306-d8` |
| GPU | NVIDIA L4 ×1 |
| 외부 IP | `136.113.193.83` (정적) |
| 컨테이너 | `js-v11-gui` (Isaac Lab), 그 외 guacamole 인프라 |

> 편의상 셸에 아래를 먼저 깔아두면 명령이 짧아짐:
> ```bash
> VM=g2-standard-16-nvidia-l4-dev
> ZONE=us-central1-c
> ```

---

## 1. VM 켜기 — 그리고 STOCKOUT ⚠️ (제일 흔한 문제)

```bash
gcloud compute instances start $VM --zone=$ZONE
```

**자주 실패함.** L4 GPU는 이 존에서 **재고가 자주 없어서** 아래 에러가 뜸:

```
ERROR: ... does not have enough resources available to fulfill the request.
'... (state: STOCKOUT, sub-state: STOCKOUT, resource type: compute)'.
```

→ **네 잘못 아님. 그냥 재고 없음.** 될 때까지 재시도하면 됨(보통 몇 분 안에 풀림).

### 🔁 "될 때까지 시도하기" — 복붙용 루프
STOCKOUT이면 2초 간격으로 **RUNNING 될 때까지 자동 재시도**:

```bash
while true; do
  OUT=$(gcloud compute instances start $VM --zone=$ZONE 2>&1)
  if echo "$OUT" | grep -qi "STOCKOUT\|does not have enough resources\|unavailable"; then
    echo "STOCKOUT — 2초 후 재시도"; sleep 2; continue
  fi
  ST=$(gcloud compute instances describe $VM --zone=$ZONE --format="value(status)")
  echo "status=$ST"
  [ "$ST" = "RUNNING" ] && { echo "✅ RUNNING"; break; }
  sleep 2
done
```

- `start` 호출 자체도 몇 초 걸려서 실질적으로 거의 연속 시도임.
- **2초보다 더 짧게 하지 말것** — gcloud API rate-limit에 걸려 오히려 느려짐.
- 한 번만 시도하고 말 거면 위 루프 대신 `start` 한 줄만.

---

## 2. SSH 접속 — "Connection refused" 대응

VM이 RUNNING이어도 **부팅 직후 20~40초**는 SSH가 안 열림:

```bash
gcloud compute ssh $VM --zone=$ZONE --command="echo ok"
# ssh: connect to host ... port 22: Connection refused   ← 정상, 아직 부팅 중
```

→ **20~30초 기다렸다 다시.** 자동 재시도:

```bash
for t in 1 2 3 4 5; do
  gcloud compute ssh $VM --zone=$ZONE --command="echo SSH_OK" 2>/dev/null | grep -q SSH_OK && { echo "✅ 접속됨"; break; }
  echo "아직 안 열림 — 20초 대기"; sleep 20
done
```

- 그래도 계속 refused면: 방화벽에서 SSH(22) 막혔는지, IAP 터널 필요한지 확인 → `--tunnel-through-iap` 붙여보기.
- 최초 접속 시 SSH 키 생성 프롬프트가 뜰 수 있음(엔터/비번 빈칸).

---

## 3. 공용 VM 예절 — 쓰기 전에 남의 작업 확인 ⚠️

**이 VM/도커는 팀 공용.** 접속하면 **먼저 남이 학습 중인지 확인**하고, 돌고 있으면 **건드리지 말고 대기**:

```bash
gcloud compute ssh $VM --zone=$ZONE --command="\
  echo '=== GPU ==='; nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader; \
  echo '=== 학습 프로세스 ==='; ps aux | grep -E 'train.py|play.py' | grep -v grep; \
  echo '=== 컨테이너 ==='; sudo docker ps --format '{{.Names}}\t{{.Status}}'"
```

- **GPU util 0% + train/play 프로세스 없음** = 유휴 → 써도 됨.
- GPU가 돌거나 train.py가 있으면 = **남이 학습 중** → 멈추고 대기(끄지도, 컨테이너 재시작도 하지 말 것).

---

## 4. 컨테이너 진입 & 작업

```bash
# 컨테이너 켜기(꺼져 있으면)
gcloud compute ssh $VM --zone=$ZONE --command="sudo docker start js-v11-gui"

# 컨테이너 안에서 명령 실행 (예: 코드 위치 확인)
gcloud compute ssh $VM --zone=$ZONE --command="sudo docker exec js-v11-gui bash -c 'ls /workspace/drone-bombard/isaac_lab'"
```

- Isaac 파이썬은 `python`이 아니라 **`/isaac-sim/python.sh`** 로 실행.
- 코드는 호스트 `~/wt-js` → 컨테이너 `/workspace/drone-bombard` 로 마운트됨. **`~/wt-js`는 git이 아니라 복사 디렉토리** → 코드 바꾸면 `gcloud compute scp`로 파일을 넣어야 반영됨.

---

## 5. 라이브스트림(WebRTC) 접속 — 화면으로 보기

```bash
# 컨테이너 안에서 play.py를 --livestream 1로 실행
gcloud compute ssh $VM --zone=$ZONE --command="sudo docker exec -d js-v11-gui bash -c \
  'cd /workspace/drone-bombard/isaac_lab && PUBLIC_IP=136.113.193.83 /isaac-sim/python.sh play.py \
   --task Isaac-DroneBombard-V19-Direct-v0 --policy <ckpt.pt> --num_envs 1 --episodes 100000 \
   --show --livestream 1 > /tmp/stream.log 2>&1'"
```

- **Isaac Sim WebRTC Streaming Client** 앱 → Server `136.113.193.83`, signaling **49100**, media **47998**.
- 방화벽 `allow-isaac-sim`(49100·47998·8211) 열려 있어야 함.
- **`--livestream 1`(public)** 써야 함 — `2`(private)는 외부에서 영상 안 옴.
- 부팅~"Streaming server started"까지 ~1분. 아무도 접속 안 하면 로그에 `nvstPushStreamData timeout`이 뜨는데 **정상**(접속하면 사라짐).

---

## 6. VM 끄기 — 과금 중단 (반드시!)

작업 끝나면 **꼭 끄기.** 켜두면 계속 과금됨.

```bash
gcloud compute ssh $VM --zone=$ZONE --command="sudo docker stop js-v11-gui"   # (선택) 컨테이너 정리
gcloud compute instances stop $VM --zone=$ZONE
# 반드시 TERMINATED 재확인:
gcloud compute instances describe $VM --zone=$ZONE --format="value(status)"
```

- `TERMINATED` 떠야 실제로 꺼진 것(=과금 중단). **stop 후 상태 한 번 더 확인**하는 습관(간혹 stop이 안 먹거나 유휴로 다시 켜져 있는 경우 있었음).

---

## 7. 자주 겪는 문제 한눈에

| 증상 | 원인 | 해결 |
|------|------|------|
| `STOCKOUT` / not enough resources | L4 재고 없음(존) | §1 재시도 루프(될 때까지). 네 잘못 아님 |
| SSH `Connection refused :22` | 부팅 직후 SSH 미기동 | 20~30초 대기 후 재시도(§2) |
| SSH 계속 안 됨 | 방화벽/IAP | `--tunnel-through-iap` 시도, 방화벽 22 확인 |
| 컨테이너서 `python: not found` | Isaac은 python.sh 사용 | `/isaac-sim/python.sh` 로 실행 |
| 코드 바꿨는데 반영 안 됨 | `~/wt-js`는 git 아님(복사본) | `gcloud compute scp`로 파일 전송 |
| 라이브스트림 영상 안 옴 | `--livestream 2`(private) | `--livestream 1` + `PUBLIC_IP` env |
| 스트림에 옛 화면/충돌 | play.py 프로세스 중복(49100) | 전부 `pkill -9 -f play.py` 후 단일 재시작 |
| 껐는데 다시 켜져 있음 | stop 미반영/유휴 재기동 | stop 후 `describe`로 TERMINATED 재확인 |

---

## 관련
- [[Environment/README]] (VM 복구 가이드) · [[sessions/commands]] · [[research/isaac_model_intro_junsang]] · [[00_index]]
