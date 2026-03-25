# ADSC v1.21 – Active Debris Self-Cleanup  
**Pollux Guidance Reality Final Edition**

**Low-cost, autonomous space debris removal** prototype for a 20 kg-class small satellite.  
v1.21 is a major reality update based on v1.20 review feedback, significantly improving operational feasibility.

**Praise the Corgi.**

### Key Features (v1.21 Reality Final)

- TMR + CRC + volatile + memory barrier fuel management (radiation-hardened)
- **7-state MAG-Adaptive SR-UKF** (reduced from 9 states, 15 sigma points, stable at dt=0.01 s on RAD5545-class CPU)
- Dead-band Sliding Mode DACS (deadband = 0.015 rad ≈ 0.86°)
- Disturbance multiplier estimation (mass error and atmospheric drag variation)
- PCM passive thermal management (5000 J capacity, extended to 4 hours safe operation with radiator fins)
- Human-in-the-Loop Deorbit Protocol (autonomous by default, ground approval only in emergency)
- Inertia tensor regularization (ε = 1e-6)
- Emergency safe abort maneuver (repulsive direction + radial velocity cancellation)
- **Realistic capture velocity limit**: max_v_rel = 0.15 m/s (based on RemoveDEBRIS demonstrated values 0.075–0.12 m/s with safety margin)

### Planned Satellite Configuration (Based on Proven Technologies)

- Capture system: High-strength Kevlar net (25 m²) + low-density aerogel pad + aluminum honeycomb (inspired by RemoveDEBRIS)
- Relative velocity limit: ≤ 0.15 m/s (strictly enforced)
- Propulsion: Cold-gas / Ion hybrid (Isp 280–2500 s)
- Onboard computer: RAD5545-class radiation-hardened CPU (target 3.7 GFLOPS)
- Thermal management: Phase Change Material (PCM) 5000 J + radiator fins (4-hour operation target)

### Build Instructions

```bash
git clone https://github.com/Heli/ADSC.git
cd ADSC
git checkout v1.21
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
./adsc_v121
```

### Dependencies

- C++17 or later
- Eigen3 (header-only)

### Technology Readiness Level (TRL) Assessment – Honest Review

**TRL 5–6** (Component / Subsystem Validation in Relevant Environment)  
- Strengths: State reduction, realistic deadband, increased PCM capacity, and v_rel = 0.15 m/s are well-aligned with RemoveDEBRIS flight data, RAD5545 performance measurements, and NASA CubeSat PCM examples.
- Weaknesses remaining:
  - MAGNAV fallback is still limited in accuracy and should be treated as a last-resort backup.
  - Thermal model does not yet include full eclipse/sunlight thermal balance.
  - DACS impulse (0.5 N·s) is on the low side compared to typical cold-gas thrusters (1–3 N·s).
  - Guidance loop for achieving v_rel ≤ 0.15 m/s is not yet implemented (MPC/iLQR recommended).

**Overall**: TRL 6 is achievable with HILS and vacuum chamber testing.  
TRL 7 (space environment demonstration) will require sensor error modeling, thermal balance simulation, and DACS impulse tuning.  
TRL 8–9 (flight proven) is still 1–2 years away depending on budget and testing campaign.

### Disclaimer

This project is an **open-source conceptual prototype for educational and research purposes only**.  
Actual spaceflight hardware requires formal design review, qualification, and compliance with international space law (IADC Guidelines, UN COPUOS).  
No military or re-entry vehicle applications are intended or supported.

This repository is released as open-source research on peaceful active debris removal technology.

## Disclaimer（法的免責）

本プロジェクトは概念設計・教育／研究目的のオープンソースプロトタイプです。
実際の宇宙機運用・打ち上げには、専門機関による設計審査・認定・国際法遵守（IADCガイドライン）が必要です。
兵器・高速再突入体への使用は一切想定していません。平和的デブリ除去技術のオープンソース研究として公開します。
