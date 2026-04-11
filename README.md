# 🤖 TurtleBot3 Autonomous Driving System

### UKF · LiDAR · DNN · NMPC 기반 지능형 자율주행 제어 시스템

> **Graduation Project** — 비선형 상태 추정 · 센서 융합 · 위험도 예측 · 최적 제어의 완전 통합 구현  
> ROS1 (Melodic / Noetic) · TurtleBot3 Burger Pi · Python · C++

[![ROS](https://img.shields.io/badge/ROS1-Melodic%2FNoetic-22314E?style=flat-square&logo=ros)](http://wiki.ros.org)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?style=flat-square&logo=python)](https://python.org)
[![C++](https://img.shields.io/badge/C++-14-00599C?style=flat-square&logo=cplusplus)](https://isocpp.org)
[![PyTorch](https://img.shields.io/badge/PyTorch-DNN-EE4C2C?style=flat-square&logo=pytorch)](https://pytorch.org)

---

## 📌 프로젝트 개요

TurtleBot3 (Burger Pi)를 플랫폼으로, LiDAR · IMU · Odometry 데이터를 활용하여  
**UKF 기반 상태 추정 → DNN 기반 위험도 예측 → NMPC 기반 최적 제어** 파이프라인을 직접 설계 및 구현한 자율주행 시스템입니다.

단순 SLAM이 아닌 **인지 → 판단 → 제어**까지 연결된 통합 자율주행 시스템으로,  
특히 **UKF를 통한 센서 노이즈 제거**가 NMPC 제어 안정성에 직접적으로 기여한다는 것을 실험을 통해 검증하였습니다.

| 항목 | 내용 |
|---|---|
| 플랫폼 | TurtleBot3 Burger Pi |
| 환경 | ROS1 (Melodic / Noetic) |
| 언어 | Python · C++ |
| 핵심 알고리즘 | UKF · DNN (PyTorch) · NMPC |

---

## 🏗️ 시스템 아키텍처

```
[Sensor Input]
  LiDAR (/scan)  │  IMU (/imu)  │  Odometry (/odom)
                 │
                 ▼
        ┌─────────────────┐
        │  UKF Estimator  │  ← 센서 융합 + 노이즈 제거
        │  (ukf_node.py)  │    상태 추정: (x, y, θ, v)
        └────────┬────────┘
                 │ /scan_ukf · /ukf_pose
                 ▼
        ┌─────────────────┐
        │  DNN Predictor  │  ← 시계열 상태 히스토리 입력
        │  (risk_model.py)│    Risk Score (0 ~ 1) 출력
        └────────┬────────┘
                 │ /risk_score
                 ▼
        ┌─────────────────┐
        │  NMPC Controller│  ← 경로 오차 + 위험도 기반 최적화
        │ (nmpc_solver.py)│    최적 제어 입력 생성
        └────────┬────────┘
                 │ /cmd_vel
                 ▼
        [TurtleBot3 Motion]
```

---

## 🎥 시연 영상

> 아래 영상을 통해 UKF 필터링 효과 및 실시간 자율주행 결과를 확인할 수 있습니다.

| # | 내용 | 링크 |
|---|---|---|
| 1 | Raw LiDAR vs UKF Filtered 비교 | 📎 (추가 예정) |
| 2 | UKF + NMPC 통합 자율주행 | 📎 (추가 예정) |
| 3 | DNN 적용 전후 제어 비교 | 📎 (추가 예정) |

---

## 📐 핵심 알고리즘 설명

### 1. UKF (Unscented Kalman Filter) — 본인 핵심 구현 파트

비선형 시스템에서 일반 EKF의 선형화 오차를 극복하기 위해 **Unscented Transform** 기반의 칼만 필터를 구현하였습니다.

**Prediction Step (예측)**
```
χ = sigma_points(x̂, P)          # Sigma point 생성
x̂⁻ = Σ Wᵢ · f(χᵢ, u)           # 비선형 운동 모델 적용
P⁻ = Σ Wᵢ · (χᵢ - x̂⁻)(...)ᵀ + Q  # 공분산 예측
```

**Update Step (보정)**
```
z = h(χ)                         # 측정 모델 적용 (LiDAR)
K = Pxz · Pzz⁻¹                  # 칼만 게인 계산
x̂ = x̂⁻ + K · (z_meas - z)       # 상태 보정
P = P⁻ - K · Pzz · Kᵀ            # 공분산 업데이트
```

- `/scan` (Raw LiDAR) → 이상값 제거 → `/scan_ukf` (필터링 완료)
- IMU + Odometry 융합으로 위치/속도/자세각 안정 추정
- 출력: `(x, y, θ, v)` → NMPC 입력으로 전달

---

### 2. DNN (Deep Neural Network) — 위험도 예측

최근 N 스텝의 상태 히스토리를 입력으로 받아, 현재 주행 상황의 위험도를 0~1 사이의 스코어로 예측합니다.

```
Input:  [x, y, θ, v, risk_feature] × N steps
Output: Risk Score (0: 안전, 1: 위험)
```

- 학습 데이터는 실제 주행 실험을 통해 수집
- NMPC 비용 함수에 Risk Score를 실시간 반영하여 장애물 회피 성능 향상

---

### 3. NMPC (Nonlinear Model Predictive Control) — 최적 제어

매 제어 주기마다 예측 구간(Horizon) 내에서 **비용 함수를 최소화하는 최적 제어 입력**을 계산합니다.

**비용 함수 (Cost Function)**
```
J = Σ [ w₁·||e_pos||² + w₂·||e_vel||² + w₃·Risk + w₄·||Δu||² ]

e_pos  : 목표 경로와의 위치 오차
e_vel  : 목표 속도와의 오차
Risk   : DNN이 출력한 위험도 스코어
Δu     : 제어 입력 변화량 (진동 억제)
```

- DNN이 초기 제어 입력값을 예측 → 최적화 수렴 속도 향상
- 속도 및 조향각 제약 조건 포함

---

## 📊 실험 결과

### UKF 적용 효과

| 지표 | UKF 적용 전 | UKF 적용 후 |
|---|---|---|
| LiDAR 거리 변동 | 0 ~ 1.2m 스파이크 빈번 | 안정적 곡선 유지 |
| 상태 추정 안정성 | Raw 센서 직접 의존 | 노이즈 제거 후 안정 추정 |
| NMPC 제어 안정성 | 불안정한 입력으로 진동 발생 | 부드러운 제어 입력 유지 |

> UKF를 통해 센서 노이즈를 제거함으로써 NMPC 입력 안정성을 확보하고, 제어 입력 진동을 크게 감소시켰습니다.

---

### DNN + NMPC 연산 성능

| 구성 | 평균 연산 시간 | 비고 |
|---|---|---|
| NMPC 단독 | 9.29 ms | 기준 |
| DNN + NMPC | 7.13 ms | **약 23% 단축** |

> DNN이 NMPC 최적화의 초기값을 예측하여 수렴 속도를 향상시키고,  
> 실시간 제어에 적합한 연산 시간을 달성하였습니다.

---

### 제어 입력 비교

```
cmd_vel (Raw NMPC)     : ──/\/\/\/\/\/──  ← 급격한 진동
cmd_vel (UKF + NMPC)   : ──╭──────╮──  ← 부드러운 전환
```

**결론:** UKF + DNN + NMPC 통합 구성이 단독 NMPC 대비 제어 안정성과 연산 효율 모두 향상됨을 확인하였습니다.

---

## 🗂️ 디렉터리 구조

```
turtlebot3-autonomous-driving/
├── ukf/
│   ├── ukf_node.py              # UKF 메인 ROS 노드
│   ├── motion_model.py          # 비선형 운동 모델 (f(x,u))
│   └── measurement_model.py     # 센서 측정 모델 (h(x))
├── lidar/
│   ├── lidar_preprocess.cpp     # LiDAR 노이즈 제거 및 이상값 필터링
│   └── obstacle_detector.cpp    # 장애물 인지 및 Feature 생성
├── dnn/
│   ├── risk_model.py            # DNN 추론 ROS 노드
│   └── train_risk_model.ipynb   # 모델 학습 노트북
├── nmpc/
│   ├── nmpc_solver.py           # NMPC 최적화 솔버
│   └── cost_function.py         # 비용 함수 정의
├── launch/
│   ├── ukf.launch               # UKF 단독 실행
│   ├── nmpc.launch              # NMPC 단독 실행
│   └── full_system.launch       # 전체 시스템 통합 실행
└── README.md
```

---

## 🔧 실행 방법

### 환경 요구사항

```bash
ROS1 Melodic 또는 Noetic
Python 3.x
PyTorch (또는 Keras)
TurtleBot3 패키지
```

### 설치

```bash
cd ~/catkin_ws/src
git clone https://github.com/seongjin-yoon/turtlebot3-autonomous-driving.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

### 실행

```bash
# 1. UKF 상태 추정기만 실행
roslaunch turtlebot3_autonomous ukf.launch

# 2. NMPC 제어기만 실행
roslaunch turtlebot3_autonomous nmpc.launch

# 3. 전체 시스템 통합 실행
roslaunch turtlebot3_autonomous full_system.launch
```

### 주요 ROS 토픽

| 토픽 | 방향 | 설명 |
|---|---|---|
| `/scan` | Input | Raw LiDAR 데이터 |
| `/imu` | Input | IMU 데이터 |
| `/odom` | Input | Odometry 데이터 |
| `/scan_ukf` | UKF Output | UKF 필터링된 LiDAR |
| `/ukf_pose` | UKF Output | 추정된 로봇 상태 (x, y, θ, v) |
| `/risk_score` | DNN Output | 위험도 스코어 (0~1) |
| `/cmd_vel` | NMPC Output | 최적 제어 명령 |

---

## 👤 기여 내역

본 프로젝트에서 **UKF 설계 및 구현을 주도**하였으며, 전체 시스템 통합까지 참여하였습니다.

| 파트 | 내용 | 역할 |
|---|---|---|
| **UKF 상태 추정** | 비선형 운동 모델 · 측정 모델 · Sigma point 설계 | **주담당** |
| **센서 융합** | IMU + Odometry + LiDAR 융합 구조 설계 | **주담당** |
| **LiDAR 전처리** | 이상값 제거 · Feature 생성 파이프라인 | 참여 |
| **DNN 연동** | Risk Score → NMPC 비용항 반영 | 참여 |
| **시스템 통합** | ROS 노드 연결 · launch 파일 제작 | 참여 |

---

## 🔮 향후 발전 방향

| 현재 | 개선 방향 |
|---|---|
| UKF | Factor-Graph 기반 Back-End 확장 |
| DNN (MLP/시계열) | Transformer 기반 위험도 예측 |
| NMPC | 강화학습 기반 Adaptive MPC |
| TurtleBot3 (소형) | 실차 규모 플랫폼 확장 |
| ROS1 | ROS2 마이그레이션 |

---

<div align="center">

Graduation Project · ROS1 Autonomous Driving · TurtleBot3 Burger Pi  
UKF · DNN · NMPC 통합 자율주행 제어 시스템

</div>
