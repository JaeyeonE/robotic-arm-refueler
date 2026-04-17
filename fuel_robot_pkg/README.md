# fuel_robot_pkg

ROS 2 기반으로 Doosan 로봇을 제어하고, 비전 결과를 이용해 자동 주유 시퀀스를 수행하는 패키지입니다.

---

## 개요

이 패키지는 전체 시스템에서 **로봇 제어와 작업 흐름 관리**를 담당합니다.

- Vision → 좌표 전달 받음
- Task Manager → 전체 시퀀스 관리
- Commander → 로봇 실제 이동
- UI Gateway → 외부(Web)와 연결
- Safety → 안전 체크

---

## 폴더 구조

```text
fuel_robot_pkg/
├── fuel_robot_pkg/
│   ├── camera.py
│   ├── doosan_commander_node.py
│   ├── fuel_port_perception_node.py.bak
│   ├── fueling_task_manager_node.py
│   ├── gripper_drl_controller.py
│   ├── safety_monitor_node.py
│   ├── ui_gateway_node.py
│   └── temp.py
│
├── launch/
│   └── fuel_robot.launch.py
│
├── resource/
├── package.xml
├── setup.py
├── requirements.txt
└── README.md
