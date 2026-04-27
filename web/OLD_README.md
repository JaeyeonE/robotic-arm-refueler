
# Demo Integration

Vision, Web, ROS2, Robot을 하나로 연결해 실제 데모 환경에서 동작하는 통합 브랜치입니다.

## Demo Youtube

### Edited process video(youtube)
https://youtube.com/shorts/2rjvqGU6aQo?feature=share

### Full process video(youtube)
https://www.youtube.com/watch?v=3dlU25bXeFo



---

## 시스템 아키텍처

<img width="1080" height="1350" alt="image" src="https://github.com/user-attachments/assets/c847ae01-6292-42b4-a808-ee9f544ef214" />


위 구조를 기준으로 시스템은 크게 4개 계층으로 나뉩니다.

### 1. 인식 계층 (Perception)
- **비전 센서**: Intel RealSense D455
- **객체 검출**: YOLOv12
- **좌표 변환**: OpenCV 기반 카메라 좌표계 → 로봇 베이스 좌표계 변환

역할:
- RGB-D 데이터 획득
- 주유구 / 주유캡 / 주유건 실시간 인식
- 로봇 제어에 필요한 좌표 계산

### 2. 제어 계층 (Control)
- **로봇 제어기**: Doosan Robotics ROS2, Python SDK
- **엔드 이펙터**: XYZ Gripper
- **작업 시퀀서**: Python Main Logic, FSM, 예외 처리 로직

역할:
- 검출된 좌표를 이용한 로봇 이동
- 주유건 파지 / 탈거
- 작업 단계별 시퀀스 제어

### 3. 데이터 계층 (Data)
- **데이터베이스**: SQLite
- **로그 관리**: Python Logging Module

역할:
- 작업 ID, 시간, 성공 여부 저장
- 비전 좌표값 저장
- 에러 코드, 동작 상태 기록

### 4. 서비스 계층 (Service)
- **대시보드**: Streamlit
- **운영 알림**: Web UI

역할:
- 작업 통계 및 성공률 확인
- 실시간 비전 피드 확인
- 비정상 상황 알림
- 작업 히스토리 조회

---

## 전체 동작 흐름

```text
1. Vision → cap_handle 및 주유 관련 객체 검출
2. 좌표 계산 및 로봇 좌표계 변환
3. Web / DB에 상태 저장
4. 사용자 UI에서 작업 시작
5. ROS2 Task Manager가 시퀀스 실행
6. Doosan Commander가 로봇 이동 수행
7. 작업 결과를 로그/DB/UI에 반영
