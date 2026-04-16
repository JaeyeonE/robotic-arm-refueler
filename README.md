일단 can_i_use_gpu.py로 torch가 자동으로 gpu 잡는지 확인해봤는데 안돼서

# 1. 드라이버 설치
sudo apt update
sudo apt install nvidia-driver-535 -y

# 2. 재부팅

# 3. torch 재설치 (cuda 12.1 버전용)
sudo pip install --force-reinstall torch torchvision --index-url https://download.pytorch.org/whl/cu121

### dataset (주유구)
https://universe.roboflow.com/fuel-cap/fuel-cap/browse?queryText=&pageSize=50&startingIndex=0&browseQuery=true

### XYZ를 좀 정해보자 
1. BASE SPACE가 어떻게 정의되어있는지 알아오기 
2-1. 카메라가 BASE SPACE로부터 얼마나 떨어져 있는지 확인하기
2-2. 카메라가 어느 방향을 바라보는지 확인하기 
3. Affine Transform 진행 (640x480 이거랑, depth 맞게 연산하는 로직 추가)
4. 잘 연동됐는지 확인해보기 (로봇을 움직일깡)


### doosan robot 연결해보기 
ros2 launch dsr_bringup2 dsr_bringup2_default.launch.py host:=110.120.1.52  port:=12345 mode:=real model:=e0509

### ...
```plain
sed -i 's/rt_host: "192.168.137.50"/rt_host: "110.120.1.2"/' ~/ros2_ws/src/doosan-robot2/dsr_controller2/config/default.yaml

sed -i 's/rt_host: "192.168.137.50"/rt_host: "110.120.1.2"/' ~/ros2_ws/install/dsr_controller2/share/dsr_controller2/config/default.yaml
```
진행했다 . . . .잊지말도록 

