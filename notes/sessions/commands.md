---
date: 2026-04-16
updated: 2026-09-25
tags: [commands, docker, isaac-lab, training, evaluation, reference]
status: active
type: reference
---

# 자주 쓰는 명령어 모음 (Isaac Lab, 2026-09 기준)

> CLAUDE.md에서 참조. Gazebo/PX4/ROS2 시절 명령은 [[research/legacy/system_overview]]·git 이력에만 남긴다.
> 코드 위치는 [[research/code_map]], 파일 역할은 [[research/isaac_lab_architecture]].

## 0. 컨테이너와 동기화

```bash
docker start isaac-verify                                   # VM 재부팅 후 (Exited 137이면 이것)
docker cp isaac_lab/. isaac-verify:/tmp/rebuild/            # 편집 후 필수 — 실행 사본은 /tmp/rebuild
docker exec -it isaac-verify bash                           # 대화형
IL=/workspace/isaaclab/isaaclab.sh                          # 컨테이너 안 파이썬: $IL -p script.py
CK1=/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt   # 본 L0 (seed 1)
```

산출물은 전부 컨테이너 `/tmp`: `/tmp/sl`(덤프·회귀기) · `/tmp/sl_GI /tmp/gru /tmp/gust /tmp/dr_axis /tmp/sl_R /tmp/ou /tmp/ema`(평가 JSON).
호스트로 가져오려면 `docker cp isaac-verify:/tmp/<dir> <local>`.

## 1. 단위테스트 (kit python에는 torch가 없으므로 isaaclab.sh 경유)

```bash
docker exec isaac-verify bash -c "cd /tmp/rebuild && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /workspace/isaaclab/isaaclab.sh -p -m pytest tests -p no:cacheprovider -q"
```

## 2. L0 비행 정책 학습 (PPO, 2048 envs, 1000 iter, 약 1시간)

```bash
docker exec -d --env-file /opt/drone-bombard/.wandb.env -e PYTHONUNBUFFERED=1 isaac-verify bash -c \
  "cd /tmp/rebuild && /workspace/isaaclab/isaaclab.sh -p train.py --task_env --no_residual \
   --dr_scale 1.5 --num_envs 2048 --max_iterations 1000 --seed 1 --headless \
   --log_root /tmp/l0b/logs > /tmp/l0_train.log 2>&1"
docker exec isaac-verify tail -f /tmp/l0_train.log
```

## 3. 잔차 지도학습 (본 방법 GRU-S) — 덤프 → 적합 → 주입

```bash
# ① 덤프: 동결 L0, 잔차 없음, 정상 바람 DR 1.5 (v2 = 원 상태·상수 포함)
$IL -p play.py --policy $CK1 --no_residual --headless --num_envs 256 --episodes 1500 \
    --marker_dist 18 22 --dr_scale 1.5 --seed 1000 --dump_sl /tmp/sl/v2_s1a.npz     # seed 2000 → v2_s1b
# ③ 적합 (GRU 26→64→2, MSE, 150 epoch, 1분)
$IL -p _fit_sl_seq.py /tmp/sl/v2_s1a.npz /tmp/sl/v2_s1b.npz --export /tmp/sl/res_gru_S.pt
# ⑤ 주입 평가 (paired, seed 3000/4000/5000 × 200)
$IL -p play.py --policy $CK1 --paired_eval --headless --episodes 200 --num_envs 200 \
    --marker_dist 18 22 --dr_scale 1.5 --seed 3000 \
    --sl_residual /tmp/sl/res_gru_S.pt --sl_ema 0.3 --arm_name GRUS --out-json /tmp/gru/GRUS_E_tau0_s3000.json
```

비교 팔: L0 `--no_residual` · 오라클 `--oracle_residual_wind_only --oracle_ema 0.3` · 규칙 팔 T0/T2 `baseline_drop.py --arm hover|argmin`.
바람 조건: 돌풍만 `--wind_tau T` · 현실 `--wind_tau 10 --wind_gust 0.2`(A) / `--wind_tau 3 --wind_gust 0.3`(B) · DR 노브 `--dr_scale`.
일괄 스크립트: `_gru_eval.sh`(τ 사다리) · `_gust_eval.sh`(현실 A·B) · `_grus_final.sh`(DR 스윕·미지 사거리) · `_ema_sweep.sh`(α 민감도).

## 4. 표·그림

```bash
$IL -p _agg_table1.py "L0=/tmp/sl_GI/L0_dr1.5_s*.json" "ours=/tmp/gru/GRUS_E_tau0_s*.json"   # 시드셋 assert
$IL -p _fig_final.py --out /tmp/figs && docker cp isaac-verify:/tmp/figs/. notes/figures/
```

## 5. 백그라운드 실행과 감시

```bash
docker exec -d -e PYTHONUNBUFFERED=1 isaac-verify bash -c "bash /tmp/rebuild/_gust_eval.sh > /tmp/gust.log 2>&1"
docker exec isaac-verify bash -c "ls /tmp/gust/*.json | wc -l; grep -c FAIL /tmp/gust.log"
docker exec isaac-verify pkill -f "[p]lay.py"        # 중단 — 대괄호 패턴으로 자기 셸 kill 방지
```

## 6. Git

```bash
git add <바꾼 파일들>          # git add . 금지 (worktree 잔여 stale 파일 있음)
git commit -m "<type>(<scope>): 요약" && git push origin main
graphify update .              # 코드 수정 후 지식 그래프 갱신 (LLM 불필요)
```

## 7. notes 목차 재생성

노트를 추가·이동하면 [[00_toc]]을 다시 만든다. 스크립트는 `notes/_make_toc.py`.

```bash
python3 notes/_make_toc.py
```
