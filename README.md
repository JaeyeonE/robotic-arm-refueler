# AutoJet Refueling System — Demo Integration

Vision, Web, ROS 2, Robot을 하나로 연결해 실제 데모 환경에서 동작하는 통합 프로젝트입니다.

## Demo Youtube

### Edited process video(youtube)
https://youtube.com/shorts/2rjvqGU6aQo?feature=share

### Full process video(youtube)
https://www.youtube.com/watch?v=3dlU25bXeFo

---

## 시스템 아키텍처

<img width="1080" height="1350" alt="image" src="https://github.com/user-attachments/assets/c847ae01-6292-42b4-a808-ee9f544ef214" />

### 1. 인식 계층 (Vision)
- **센서**: Intel RealSense D435i
- **인식 모델**: YOLOv8 (Custom trained for Gas Cap & Handle)
- **역할**: 주유구 위치 검출, 주유기 핸들 각도 추정 및 로봇 좌표계 변환

### 2. 제어 계층 (Control)
- **로봇 제어기**: Doosan Robotics ROS 2, Python SDK
- **엔드 이펙터**: XYZ Gripper
- **작업 시퀀서**: ROS 2 Action 기반 (\`FuelTask\` Action Server), 단일 동작 단위로 분리
- **역할**: 검출된 좌표를 이용한 로봇 이동, 그리퍼 제어, 충격 감지 및 비상 정지 로직

### 3. 통신 및 데이터 계층 (Communication)
- **미들웨어**: ROS 2 Humble
- **데이터베이스**: SQLite (WAL 모드 지원)
- **로깅**: 시스템 전역 로그 및 작업 세션 통계 수집

### 4. 서비스 계층 (Service)
- **백엔드 / API**: Flask (Python) + SQLite
- **사용자 UI**: 고객용 키오스크 및 관리자용 모니터링 대시보드 (Vanilla HTML/CSS/JS)
- **역할**: 작업 통계 확인, 실시간 모니터링, 사용자 명령 전달

---

## 작업 시나리오 흐름
\`\`\`text
1. Vision 모듈이 주유구 검출 및 좌표 계산
2. 좌표 정보가 Web / DB에 실시간 저장
3. 고객이 키오스크에서 주유 시작 명령 전송
4. Web 서버가 ROS 2 Gateway를 통해 Action Goal 송신
5. 로봇 제어기가 Doosan Commander를 통해 동작 수행 (Step별 피드백 전송)
6. 작업 완료 후 결과(로그, 통계, 캡처)가 DB 및 UI에 최종 반영
\`\`\`

---

## 모듈별 상세 정보

- [**vision/**](./vision/README.md) — RealSense + YOLO 비전 인식 모듈
- [**web/**](./web/README.md) — Flask 백엔드 + 키오스크/관리자 UI + 데이터베이스
- [**fuel_robot_pkg/**](./fuel_robot_pkg/README.md) — ROS 2 로봇 제어 패키지
- [**fuel_robot_interfaces/**](./fuel_robot_interfaces/README.md) — 커스텀 메시지 및 액션 정의

---
