---
date: 2026-07-23
tags: [error, wandb, docker, isaac-lab]
status: resolved
type: error
---

# err_20260723 — 컨테이너 baked-in WANDB_API_KEY 공백 → 학습 silent 실패

## 증상

`--logger wandb` 학습(exp_020 1차 기동)이 기동 ~1분 만에 종료.
`wandb.errors.errors.UsageError: No API key configured` Traceback이 로그에 있으나
**`isaaclab.sh`는 exit 0** — 배치/오케스트레이션 관점에선 "성공"으로 보임(기지 함정,
Rule: isaaclab.sh 실패 삼킴 → 로그 grep 감시).

## 원인

`isaac-verify` 컨테이너 생성 시 `WANDB_API_KEY`가 **빈 값으로 baked-in**
(`docker inspect` Config.Env에 `WANDB_API_KEY=` 존재 → 있어 보이지만 length 0).
`docker exec`는 컨테이너 Config.Env를 상속하므로 빈 키가 그대로 전파.

## 해결

학습/평가 exec에 항상 호스트 키 파일을 명시 주입:

```bash
docker exec -u root --env-file /opt/drone-bombard/.wandb.env -e PYTHONUNBUFFERED=1 isaac-verify …
```

(호스트 `/opt/drone-bombard/.wandb.env`, 키 86자 — Isaac Container Access 메모리 규칙과 동일.)

## 재발 방지

- wandb 학습 기동 전 체크: `docker exec <c> bash -c 'echo ${#WANDB_API_KEY}'` → 0이면 `--env-file` 필수.
- 기동 직후 로그에서 `View run at`(wandb URL) 라인 확인 — 없으면 init 실패로 간주.

## 관련

- [[experiments/exp_020_o5jn9xzk_payload_training]]
- [[research/rl_rules]] — isaaclab.sh exit-0 삼킴 감시 규칙
