# Situation Awareness Software ver1.1 (UAM demo)

UAM(도심항공교통) 운용을 가정한 상황인식(Situation Awareness) 코드입니다.

- 센서(자체 시뮬레이터)로부터 Ownship/Traffic(타 항공기/드론) 측정값을 생성
- 트래커(α-β 필터)로 트랙 유지
- CPA(Closest Point of Approach) 기반 충돌위험 평가
- 위험 이벤트를 JSON 로그로 출력 + 선택적으로 UDP 송신


## Build (Linux/WSL/Ubuntu)
```bash
sudo apt-get update
sudo apt-get install -y cmake build-essential

mkdir -p build
cmake -S . -B build
cmake --build build -j
./build/uam_sa_demo --rate 20
```

## Windows (MSVC)
```powershell
cmake -S . -B build
cmake --build build --config Release
./build/Release/uam_sa_demo.exe --rate 20
```

## Run options
```bash
./uam_sa_demo --rate 20 --duration 15
./uam_sa_demo --udp 127.0.0.1 18000
```
