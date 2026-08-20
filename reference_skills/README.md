# 참고용 ROS2/Nav2 스킬 모음

parking_mission 코드 작성 전 참고자료로 다운받은 외부 저장소들. **여기 있는 내용은
일반적인 Nav2/ROS2 지식**이고, 자이카 Y모델 고유의 실측값(휠베이스, 조향각 단위,
`/xycar_motor` 토픽 규약 등)은 들어있지 않다 — 그건 track_parking/MEASUREMENTS.md
쪽을 봐야 한다.

## ros2-copilot-skills/ (158개 스킬, GitHub Copilot 포맷)

https://github.com/wimblerobotics/ros2-copilot-skills

이번 미션과 직접 관련된 것들:

- `smac-planner-hybrid-a-star/SKILL.md` — Hybrid-A* 플래너 설정. Ackermann +
  후진 필요한 우리 상황과 정확히 일치. `motion_model_for_search: REEDS_SHEPP`,
  `allow_reverse_expansion`, `minimum_turning_radius`, `reverse_penalty` 등
  파라미터 표와 트러블슈팅 포함.
- `regulated-pure-pursuit/SKILL.md` — RPP 컨트롤러. **`use_rotate_to_heading`와
  `allow_reversing`는 상호 배타적**이라는 게 명시되어 있음 (기존 track_parking
  설정이 `use_rotate_to_heading` 안 건드리고 있었는데 기본값이 뭔지 확인 필요).
- `amcl-tuning/SKILL.md` — AMCL 파라미터 해석. alpha1~4 튜닝 전략.
- `wheel-odometry-model/SKILL.md`, `motor-controller-interface/SKILL.md` —
  오도메트리/모터 인터페이스 일반론. vesc_odom_bridge 개선 시 참고.
- `costmap-architecture/`, `global-costmap-config/`, `local-costmap-config/`,
  `localization-recovery/` — 코스트맵/복구 튜닝.

전체 목록은 `ls ros2-copilot-skills/` 로 확인. Copilot용 SKILL.md 포맷이라
Claude Code가 자동 로드하지는 않음 — 필요할 때 직접 Read해서 참고하는 용도.

## ros2-engineering-skills/ (Claude Code 네이티브 플러그인)

https://github.com/dbwls99706/ros2-engineering-skills

`references/navigation.md`에 Nav2/SLAM/AMCL/costmap/BT navigator/collision
monitor를 한 파일로 정리. Claude Code 플러그인으로 설치 가능:

```
/plugin marketplace add dbwls99706/ros2-engineering-skills
/plugin install ros2-engineering@ros2-engineering-skills
```

(아직 설치는 안 한 상태 — 저장소만 로컬에 받아둠)

## 안 받은 것

- `arpitg1304/robotics-agent-skills` — SLAM/Nav2/parking 관련 스킬 전무
  (로드맵에만 있고 미출시), 다운 생략.
- `jherrodthomas/robotics-skills-suite` — 산업용 로봇/안전인증(ISO/IEC) 중심,
  이 미션과 무관해서 생략.
