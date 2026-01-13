# 🍔 ROBURGER ASSEMBLY SYSTEM (햄버거 제조 자동화 시스템)

> **"자동화 최적화"**를 통해 패스트푸드 산업의 노동 강도를 줄이고 위생과 효율을 극대화하는 로봇 솔루션

![Project Banner](https://via.placeholder.com/1000x300?text=ROBURGER+ASSEMBLY+SYSTEM)
## 📖 Project Overview (프로젝트 개요)

**ROBURGER**는 패스트푸드점의 높은 노동 강도와 위생 문제를 해결하기 위해 고안된 협동로봇 기반 햄버거 조립 자동화 시스템입니다. 기존의 패티 조리 특화 로봇(예: 알파그릴)과 달리, 패티 조리뿐만 아니라 번(Bun), 치즈, 야채, 소스 등 햄버거 제조의 전 과정을 자동화하여 차별화된 솔루션을 제공합니다.

### 🎯 Key Objectives (핵심 목표)
* **Full Automation:** 패티 조리부터 햄버거 조립 및 포장까지 전 공정 수행.
* **Safety First:** 작업자 피로 감소 및 협동로봇 안전 기준 준수.
* **Efficiency:** 자동화를 통한 품질 일정화 및 ROI(투자 대비 수익) 상승.

---

## 🛠 Tech Stack (기술 스택)

### Environment
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu) ![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros) ![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)

### Backend & Database
![Firebase](https://img.shields.io/badge/Firebase-Realtime_DB-FFCA28?logo=firebase) ![ROS Bridge](https://img.shields.io/badge/ROS-Bridge-22314E)

### Frontend & UI
![React](https://img.shields.io/badge/React-UI-61DAFB?logo=react) ![JavaScript](https://img.shields.io/badge/JavaScript-ES6-F7DF1E?logo=javascript)

### Hardware
* **Robot:** Doosan Robotics M0609 (Collaborative Robot).
* **Gripper:** Custom 3D Printed Grippers & Spatula.
* **Equipment:** 재료 스테이션, 조립대, 소스/집게/뒤집개 거치대.

---

## ⚙️ System Architecture (시스템 구조)

시스템은 **ROS2 기반의 로봇 제어**, **Firebase 실시간 DB**, 그리고 **사용자/관리자 UI**로 구성되어 유기적으로 통신합니다.

1.  **UI (User Interface):**
    * **Customer:** 키오스크 형태의 메뉴 주문 화면.
    * **Admin:** 로봇 상태 모니터링, 긴급 정지, 매출 통계, 미니 조그(Jog) 제어.
2.  **Server (Firebase & Bridge):**
    * 주문 데이터(`get order data`)와 로봇 상태(`robot_status`)를 실시간으로 동기화.
    * JSON 및 std_msgs 형식으로 ROS2 토픽 통신.
3.  **Robot Control (ROS2):**
    * `/burger_job`: 햄버거 제조 시퀀스 수행.
    * `/robot_stop`: 긴급 정지 및 충돌 감지 시 안전 모드 진입.
    * `/robot_recovery`: 홈 복귀 및 작업 재개 로직 수행.

---

## 🍔 Workflow & Features (주요 기능)

### 1. Automated Assembly Process (자동 조립 공정)
로봇은 다음과 같은 순서로 햄버거를 제작합니다:
1.  **주문 수신:** 대기 위치에서 주문 확인 후 도구 스테이션 이동.
2.  **패티 & 치즈:** 뒤집개를 장착하여 패티를 뒤집고, 치즈를 올려 조립대로 이동.
3.  **야채 조립:** 집게로 툴을 교체(`Tool Change`)하여 양상추와 토마토를 적층.
4.  **소스 작업:** 소스 통을 들어 밑빵 위에 소스 도포.
5.  **마무리:** 윗빵을 덮고 포장 작업을 수행.

### 2. Safety & Recovery (안전 및 복구 시스템)
* **긴급 정지 (Emergency Stop):** 외부 충격이나 비상 버튼(UI/하드웨어) 입력 시 로봇 즉시 정지.
* **복구 모드 (Recovery Mode):** 정지 후 작업자가 '초기화(홈 이동)' 또는 '다시 시작(재개)'을 선택하여 안전하게 복구 가능.
* **안전 구역 (Safety Zone):** 협동로봇의 안전 기능을 활용, 충돌 민감도를 90%로 설정하고 속도를 80% 감속하여 운영.

### 3. Smart Dashboard (스마트 대시보드)
* 일별/메뉴별 매출 통계 자동 집계.
* 주문 현황(대기, 조리 중, 완료) 실시간 시각화.

---

## 🚀 Troubleshooting (문제 해결)

### Issue: 윗빵 내려놓기 실패
* **문제:** 윗빵을 집게로 잡고 내려놓을 때, 빵이 집게 표면에 붙어 떨어지지 않는 현상 발생.
* **해결:**
    1.  집게가 빵의 정중앙이 아닌 **살짝 뒤쪽**을 잡도록 그리핑 포인트 수정.
    2.  빵을 놓을 때 벽면에서 **5mm 이격**시켜 마찰력을 감소시킴으로써 부드럽게 안착 유도.

---

## 👥 Team Members (팀 소개 - F1 Team)

| 이름 | 역할 (Role) | 담당 업무 (Responsibilities) |
|:---:|:---:|:---|
| **김효원** | Environment / QA | 프로젝트 환경 구성, 시스템 디버깅 및 고도화 |
| **이효원** | UI / Frontend | UI 구성 및 Firebase 연동, 통합 테스트 |
| **전형준** | System / ROS | ROS 환경 통합, 예외 처리 로직, DB 구축 |
| **황혜인** | Test / Hardware | 프로젝트 환경 구성, 시스템 통합 검증, 재료 그립 모션 구현 |

---

## 📝 License & References
This project was developed as part of the ROKEY BOOT CAMP.

* **Robot:** Doosan Robotics M0609
* **Reference:** "자동화 최적화"

---
