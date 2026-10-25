# 단일 모터 스텝 응답 벤치 시험

프롭을 **한 모터에만** 달고 PX4 액추에이터 테스트로 스텝을 주며 노트북 마이크로 녹음해, 명령 → PWM 지연과 로터 회전수 응답(데드타임, 시정수)을 잰다.
결과와 해석: `notes/experiments/exp_044_f450_motor_step_response.md`.

## 준비
- Windows 호스트에서 실행한다(Pixhawk USB 포트와 마이크가 WSL에서는 보이지 않는다). `pip install pymavlink pyserial numpy sounddevice`.
- 기체를 작업대에 고정하고, 시험 모터 외에는 프롭을 뗀다. 배터리를 연결하고 QGC를 닫는다.
- 노트북은 프롭 회전면 밖, 모터 가까이에 둔다.
- 마이크 장치 번호는 `python -c "import sounddevice as sd; print(sd.query_devices())"`에서 **Windows WASAPI** 장치를 고른다.
  기본(MME) 경로는 Windows 잡음 억제가 일정한 모터 음을 지운다.

## 실행
```
python motor_step.py --motor 2 --out data/run3_motor2          # 5 s 카운트다운 후 30 % <-> 50 % × 8회
python analyze_motor_step.py data/run3_motor2
```
`--motor`는 PX4 모터 번호(1..)다. 출력 함수 번호(101..)가 아니다.

## 데이터 (2026-10-25)
| 파일 | 내용 |
|---|---|
| `data/run3_motor2_*` | 모터 2(CCW), 15.45 V |
| `data/run4_motor3_*` | 모터 3(CW), 15.44 V |

`*_audio.wav`는 48 kHz 16-bit 모노, `*_meta.json`은 명령·ACK 시각, `SERVO_OUTPUT_RAW`(호스트 수신 시각, FC 시각, PWM), 오디오 블록 콜백 시각이다. 시각은 모두 호스트 `perf_counter` 기준이다.
