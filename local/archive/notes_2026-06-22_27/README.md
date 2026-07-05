# Archived notes — 2026-06-22 ~ 06-27 (Claude 자율 진행 노트 + 정리 시점 작성 문서)

이 폴더의 내용은 **로컬 워크플로우 와 맞지 않게 Claude 가 작성** 한 것입니다.
정리 후 모든 정보는 **이미 local/ 의 정식 워크플로우 경로로 분산 정리** 완료되어 있으므로,
이 폴더는 자율 진행 history 보존용 — **읽을 필요 없습니다**.

## 원래 위치 → 정리된 위치 매핑

| 이 폴더의 파일 | 정리된 정식 위치 |
|---|---|
| `errors/err_20260622_ang_vel_callback.md` | `local/issues/issue_024_ang_vel_callback_disabled.md` |
| `experiments/exp_006_zjexq20k_v9a_payload_dist_angaccel.md` | `local/parameter_log.md §4 #38` + `local/meeting_notes/meeting_notes_2026-06-26.txt` |
| `research/dgui_tool.md` | `local/guides/dgui_usage_guide.md` |
| `research/toss_strategy_analysis.md` | `local/issues/issue_026_toss_environment_indistinguishable.md` + `local/design/design_review_2026-06-27.md` |
| `sessions/session_2026-06-21_to_27.md` | `local/meeting_notes/meeting_notes_2026-06-22.txt`, `_2026-06-26.txt`, `_2026-06-27.txt` |
| `daily/daily_2026-06-22.md` | `local/meeting_notes/meeting_notes_2026-06-22.txt` |
| `daily/daily_2026-06-27.md` | `local/meeting_notes/meeting_notes_2026-06-27.txt` |
| `diag_ang_vel_2026-06-22.md` (자율 진행 보고) | `local/issues/issue_024_ang_vel_callback_disabled.md` 의 자세 부분 |
| `v9a_train_start_2026-06-22.md` (학습 시작 plan) | `local/parameter_log.md §4 #38` + `local/meeting_notes/meeting_notes_2026-06-22.txt` |

## 작성 배경

2026-06-22 사용자 수면 중 Claude 가 ang_vel callback fix 자율 진행 → 보고서 2 개 작성 (`local/notes/` 에).
2026-06-27 사용자 "문서 정리해" 요청 → Claude 가 잘못 이해해서 처음에 `notes/` (Obsidian vault) 에 9 개 노트 작성. 사용자가 정정 ("notes 는 최종 목적지, local 에서 작업"), `local/notes/` 로 이동. 그러나 `local/` 의 정식 워크플로우 (design/, issues/, guides/, meeting_notes/, parameter_log.md) 에 맞지 않음. 다시 정정해서 정식 위치로 분산 정리. 이 폴더는 그 history 보존용.

자세히: `local/meeting_notes/meeting_notes_2026-06-27.txt` 의 PART 5 (문서 정리 작업) 참조.
