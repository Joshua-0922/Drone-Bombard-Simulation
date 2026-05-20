---
date: 2026-05-20
tags: [error, spin-thread, recursion, rclpy, reset, cruise-timeout]
status: resolved
type: error
---

# Spin Thread Death → Recursive reset() Crash

## 발생 환경
- Run: `06x7kpot` (2026-05-19 14:27 시작)
- Timestep: ~116,956 / 2,000,000 에서 크래시
- Exit code: 255

## 오류 요약

두 가지 오류가 연쇄적으로 발생:

### 1차 원인: Thread-2 (ROS2 spin) 사망
```
Exception in thread Thread-2 (spin):
  File "rclpy/executors.py", line 780, in wait_for_ready_callbacks
    return next(self._cb_iter)
```
`rclpy.spin()`이 bare 스레드에서 실행 중이었으며, DDS/RMW 레이어 예외 발생 시 스레드가 조용히 종료됨.
→ ROS2 콜백 처리 중단 → `mission_state` 영구 정지 ('IDLE' 또는 마지막 값)

### 2차 원인: reset() 무한 재귀
```
File "drone_drop_env.py", line 507, in reset
    return self.reset(seed=seed, options=options)
  [Previous line repeated 86 more times]
```
3회 CRUISE timeout 후 `self.reset()` 재귀 호출이 guard 없이 반복됨.
86단계 재귀 후 KeyboardInterrupt / RecursionError로 프로세스 종료.

## 발생 흐름
```
Thread-2 spin 예외 → rclpy 콜백 중단
→ mission_state 'CRUISE'로 절대 안 됨
→ _wait_for_cruise() x3 timeout
→ reset() line 507 재귀 호출 (guard 없음)
→ 86회 반복 → 스택 오버플로 → exit 255
```

## 수정 내용 (`drone_drop_env.py`)

### Fix 1: `_spin_loop()` 메서드 추가
`rclpy.spin()`을 try/except로 감싸는 wrapper 메서드. 예외 발생 시 로그 출력 후 spin 재시작.
`rclpy.ok()` False이거나 노드가 destroy된 경우에만 루프 종료.

### Fix 2: spin 스레드를 `_spin_loop` 기반으로 교체
`threading.Thread(target=rclpy.spin, ...)` → `threading.Thread(target=self._spin_loop, ...)`

### Fix 3: `reset()` 재귀 guard 추가
`self._reset_depth` 카운터로 최대 2회 재귀 허용.
초과 시 `RuntimeError` raise → SB3가 오류를 명확히 받도록.

### Fix 4: `reset()` 진입 시 spin 스레드 생존 확인
`self._spin_thread.is_alive()` 체크 → 죽어 있으면 새 `_spin_loop` 스레드 시작.

### Fix 5: `_wait_for_cruise()` 내 spin 스레드 체크
루프마다 `self._spin_thread.is_alive()` 확인 → 죽으면 즉시 return (caller에서 timeout 처리).

## 관련 노트
- [[experiments/training_history]]
- [[research/rl_rules]]
