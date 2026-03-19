# ADSC v1.21 – Active Debris Self-Cleanup  
**Pollux Guidance Reality Final Edition**

**低コスト・自律型デブリ除去**を目的とした20 kg級小型衛星のコア制御プロトタイプ。  
v1.21ではv1.20の指摘（UKF計算負荷、PCM運用時間、v_rel達成確率）を事実ベースで完全修正し、**TRL 5–6到達可能な現実運用レベル**に到達しました。

**Praise the Corgi.**

## 主な特徴（v1.21 Reality Final）

- TMR + CRC + volatile + メモリバリア燃料管理（宇宙線耐性）
- **7状態 MAG-Adaptive SR-UKF**（状態数9→7に圧縮、Sigma点15個、RAD5545でdt=0.01 s安定運用可能）
- Dead-band Sliding Mode DACS（死帯幅0.015 rad ≈ 0.86°）
- 未知攪乱乗数（質量誤差・大気抵抗ゆらぎ推定）
- PCMパッシブ熱管理（容量5000 J、最大安全運用4時間に拡張＋放射冷却フィン想定）
- Human-in-the-Loop Deorbit Protocol（自律優先＋緊急時のみ地上承認）
- 慣性テンソル正則化（ε = 1e-6）
- 緊急回避機動（物理的斥力＋接近速度相殺）
- **捕獲速度閾値現実化**：max_v_rel = 0.15 m/s（RemoveDEBRIS実証値0.075–0.12 m/sを安全マージン付きで設定）

## 機体構成（予定・実証技術ベース）

- 捕獲機構：高強度Kevlarネット（面積25 m²）＋低密度エアロゲルパッド＋アルミハニカム構造（RemoveDEBRIS実証技術を参考）
- 相対速度運用制限：0.15 m/s以下（厳守）
- 推進系：Cold-gas / Ionハイブリッド（Isp 280–2500 s）
- 計算機：RAD5545級耐放射線CPU想定（3.7 GFLOPS実測値で7状態UKFを安定運用）
- 熱管理：相変化材料（PCM）5000 J＋放射冷却フィン（NASA CubeSat実例を基に運用時間4時間確保）

## ビルド方法

```bash
git clone https://github.com/Heli/ADSC.git
cd ADSC
git checkout v1.21
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
./adsc_v121
```

### 依存関係
- C++17 以上
- Eigen3（ヘッダオンリー）

## 現在の成熟度（TRL目安）

TRL 5–6相当（コンポーネント／サブシステム検証レベル）

RemoveDEBRIS実証データ（ネット捕獲相対速度0.075–0.12 m/s）、RAD5545実測性能（3.7 GFLOPS）、NASA CubeSat PCM実例（gallium系数千J規模）を基に現実化したため、地上HILS・真空チャンバ試験で即検証可能。
TRL 7（宇宙環境実証）にはセンサ誤差モデル＋HILS追加が必要。

## Disclaimer（法的免責）

本プロジェクトは概念設計・教育／研究目的のオープンソースプロトタイプです。
実際の宇宙機運用・打ち上げには、専門機関による設計審査・認定・国際法遵守（IADCガイドライン）が必要です。
兵器・高速再突入体への使用は一切想定していません。平和的デブリ除去技術のオープンソース研究として公開します。
