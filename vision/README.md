# neo_post_vision.py 쓰기. (post_vision.py 보다 전처리 강화)

# Vision Module

YOLO + RealSense 기반으로 `cap_handle` 및 주유 관련 객체를 인식하고,
3D 좌표를 계산하여 웹 서버 및 ROS 시스템으로 전달하는 모듈입니다.

---

## 개요

이 모듈은 다음을 수행합니다.

1. YOLO로 객체 검출
2. depth 값을 이용해 3D 좌표 계산
3. 좌표를 HTTP로 웹 서버에 전송
4. (옵션) ROS2에서 사용할 수 있도록 확장 가능

---

## 폴더 구조

```text
vision/
├── post_vision.py         # 기본 비전 처리
├── neo_post_vision.py     # 개선 버전 (좌표/각도 안정화 등)
├── yolov12_0415.pt        # YOLO 모델
├── README.md
