# parking_mission

자이카 Y모델 자율주행 주차 미션.
출발 → **A구역 T자 후진주차** → 탈출 → **B구역 평행주차** → 탈출 → 출발지점 복귀.

## 설계 요약

미션을 두 층으로 나눈다.

| 구간 | 담당 | 이유 |
|---|---|---|
| 출발지 → 슬롯 진입지점 | **Nav2** (Smac Hybrid-A* + RPP) | 장애물 회피와 전역 경로는 Nav2가 잘한다 |
| 진입지점 → 슬롯 안 최종 정차 | **직접 구현** (`geometry.py` + `motion.py`) | 좁은 슬롯 안 정밀 기동은 Nav2 영역 밖 |

핵심은 **"도착한 자리에서 다시 푼다"** 이다. 주차 기동은 미리 만들어둔 시퀀스를
재생하는 게 아니라, Nav2가 데려다준 그 순간의 실제 위치를 읽어서 슬롯 중심까지
가는 해를 매번 새로 계산한다. 그래서 Nav2 도착 오차가 주차 결과로 전달되지 않는다.

## 파일 구성

```
parking_mission/
├── geometry.py       기동 기하 solver (ROS 무관, 단독 테스트 가능)
├── motion.py         기동 실행기 (odom 폐루프)
├── mission_config.py 공문 좌표 + 슬롯별 튜닝값
├── cmd_vel_bridge.py Nav2 /cmd_vel -> /xycar_motor
└── mission_manager.py 미션 상태기계 (핵심 노드)
params/nav2_params.yaml
launch/bringup.launch.py    지도/AMCL/Nav2/브릿지/RViz
launch/mission.launch.py    미션 주행
maps/parking_map.{pgm,yaml} 주최측 배포 지도
```

`geometry.py`는 ROS 없이 그대로 돌려서 검증할 수 있다:

```bash
python3 parking_mission/geometry.py
```

## 시스템 구성 (ROS1 + ROS2 혼합)

모터/VESC는 ROS1에 그대로 두고 `ros1_bridge`로 연결한다.

```
[ROS1]  모터 드라이버 · VESC 드라이버 · vesc_odom_publisher.py
             |                                    ^
             |  /vesc_odom(Odometry), tf          |  /xycar_motor
             v                                    |  (Float32MultiArray)
        ============ ros1_bridge ============
             |                                    ^
             v                                    |
[ROS2]  AMCL · Nav2 · cmd_vel_bridge · mission_manager
```

세 토픽 다 표준 메시지 타입이라 별도 매핑 설정 없이 브릿지를 통과한다.
(`vesc_msgs/VescStateStamped`는 커스텀이라 못 넘어가지만, ROS1 안에서만 쓰이므로 무관)

## 실행 순서

**1) ROS1 쪽** (기존 방식 그대로)

```bash
roscore
```

```bash
roslaunch vesc_odom_bridge vesc_odom.launch
```

**2) 브릿지**

```bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

**3) ROS2 브링업** — 여기까지가 "준비". 차는 아직 안 움직인다.

```bash
ros2 launch parking_mission bringup.launch.py
```

RViz에서 반드시 눈으로 확인할 것:
- 지도가 뜨는가
- AMCL 파티클이 출발지점(1.8, 0.9)에 모여 있는가
- `/scan`(빨간 점)이 지도의 벽과 겹치는가 ← 이게 어긋나면 절대 출발시키지 말 것
- 노란 footprint가 차 크기로 보이는가

**4) 미션 시작**

```bash
ros2 launch parking_mission mission.launch.py
```

```bash
ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"
```

노드를 미리 띄워놓고 심판 신호에 맞춰 위 명령으로 출발시키는 구조다.
`autostart:=true`를 주면 노드 기동 즉시 출발한다.

## 모터 토픽 충돌 방지

`/xycar_motor`를 두 노드가 쓴다 — Nav2 이동 중엔 `cmd_vel_bridge`, 슬롯 안
기동 중엔 `mission_manager`가 직접. 동시에 쓰면 모터를 두고 싸운다.

`mission_manager`가 `/nav_drive_enable`(Bool, latched)로 브릿지를 켜고 끈다.
주차 기동 직전에 false를 보내고 정지 명령을 한 번 명시적으로 박은 뒤 넘어간다.
Nav2가 목표 도달 후 알아서 조용해질 거라는 가정에 기대지 않는다.

## 현장 캘리브레이션

`mission.launch.py`에 인자로 준다.

```bash
ros2 launch parking_mission mission.launch.py radius_scale:=1.10 park_speed:=0.25
```

**`radius_scale`이 가장 중요하다.** 실제 회전반경이 설계값보다 크면(타이어 슬립으로
덜 꺾이면) 1보다 크게 올린다. 시뮬레이션 검증 결과:

| radius_scale | A구역 종방향 오차 | B구역 종방향 오차 |
|---|---|---|
| 1.00 (미보정) | -7.8 cm | -6.6 cm |
| 1.10 (실제와 일치) | **0.0 cm** | -3.8 cm |

조정 방법: 주차를 한 번 시켜보고 로그의 `종/횡 오차`를 본다. 원호가 설계보다
크게 그려져 바깥으로 밀리면 `radius_scale`을 올린다.

## 검증 상태

- `geometry.py` 자체 테스트: 해석해 잔차 1e-16 (부동소수점 한계)
- 실측 지도 충돌 스윕: A구역 최소여유 0.20 m, B구역 0.25 m
- 자전거모델 폐루프 시뮬: 진입오차 ±12 cm/±7° 조건에서 최종오차 3 cm 이내
- 반경 10% 오차 + 서보 0.3 s 지연 열화 조건: 보정 2회로 8 cm 이내 수렴

**아직 실차로는 한 번도 안 돌려봤다.** 위는 전부 시뮬레이션/기하 검증이다.

## 현장에서 반드시 확인해야 하는 가정값

`mission_config.py`에 TODO로 표시해둔 것들.

1. **`SLOT_B.lateral = 0.70`** — 통로 폭에서 역산한 값. 실제 주행 차선 위치가
   정해지면 반드시 바꿔야 한다.
2. **`SLOT_A.side = +1`** — 슬롯을 지나쳐 북쪽으로 올라간 뒤 좌조향 후진으로
   진입하는 방향. 실제 통로 진입 동선을 보고 확정할 것.
3. **충돌 여유값** — 전부 "AMCL 오차 0" 가정이다. 실제로는 위치추정 오차만큼
   깎인다. A구역은 진입 자세에서 앞코와 북쪽 벽(y=5.35) 간격이 병목이라
   특히 주의.
4. **주차칸은 지도에 안 나온다** — 바닥 테이프로만 표시되어 라이다로는 안 보인다.
   즉 주차 정확도는 전적으로 AMCL 위치추정 정확도에 달려 있다. 이게 이 미션의
   가장 큰 리스크다.

## 참고 스킬 문서의 오류 (정정해서 반영함)

`.claude/skills/smac-planner-hybrid-a-star/SKILL.md`에 두 가지 오류가 있어
`nav2_params.yaml`에서는 Humble 실제 기준으로 바로잡았다.

1. 플러그인 이름을 `nav2_smac_planner::SmacPlannerHybridAstar`라고 적었지만
   Humble의 실제 등록명은 `nav2_smac_planner/SmacPlannerHybrid`다.
2. `allow_reverse_expansion`을 Hybrid 파라미터로 적었지만 실제로는
   **SmacPlannerLattice 전용**이다. Hybrid에서 후진을 켜는 스위치는
   `motion_model_for_search: "REEDS_SHEPP"` 하나뿐이다. Hybrid에
   `allow_reverse_expansion`을 넣으면 조용히 무시되므로 "켰다고 생각했는데
   안 켜진" 상태가 되기 쉽다.
