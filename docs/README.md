# ドキュメント

DOF Reality H3 のオリジナル制御基板が故障したため、制御部を M5Stack CoreS3 ×3 で置き換えるプロジェクトの記録。

内容は、実機のテスター測定・写真・公開資料をもとに ChatGPT と行った調査・議論
（[共有リンク](https://chatgpt.com/share/6add998d-7300-83e8-ba36-f8713142507f)）を整理したもの。
各ページでは **確定（実測）／推定／未確定** を区別して書いている。推定・未確定の項目を設計の前提にする前に、必ず実機で確認すること。

## フォルダ構成

| フォルダ | 内容 |
| --- | --- |
| [h3-original/](h3-original/README.md) | DOF Reality H3 の既存構成・実装の調査結果。電源基板、制御基板、モータードライバと Protection Board、モーターとフィードバックセンサー、ブレーキ、故障診断の経緯 |
| [cores3-controller/](cores3-controller/README.md) | 作成中の M5Stack CoreS3 による 3 系統独立制御系の設計。全体アーキテクチャ、ハードウェア配線と GPIO 割当、ファームウェア設計、安全設計、進捗と未確定事項 |
| [control-server/](control-server/README.md) | PC 側コントロールサーバー h3-control-server。計画（architecture.md）、PC ⇄ CoreS3 USB プロトコル v1（protocol.md）、使い方・試験・e2e 記録、実機 bring-up 手順（[hardware-bringup.md](control-server/hardware-bringup.md)）、**ゲーム（アプリ）側の制御 tips（[game-tips.md](control-server/game-tips.md)）**、AI 向けのデバッグ手順（[ai-debugging.md](control-server/ai-debugging.md)）。PC 側は実装済み・模擬軸でのみ試験済み |

## 関連するコード

| パス | 内容 |
| --- | --- |
| `moter-controler/` | CoreS3 1 台で MD13S・主電源リレー・ブレーキリレーをタッチ操作とシリアルコマンドで動かす最初のプログラム |
| `encoder-tester/` | 主電源投入・ブレーキ解除・有限時間の PWM 駆動とエンコーダ計数を同じ時系列で記録する単軸試験ツール（別フォルダ。`docs/runs/` に実施記録） |
| `control-server/` | h3-control-server（Rust）。Game API・管理 API、3 軸の校正・プランナー・状態機械、USB プロトコル v1、MCU 参照モデル（`src/sim_axis.rs`）。設計は `control-server/DESIGN.md` |
| `clients/python/` | Game API の Python クライアント `h3_motion_client.py`（標準ライブラリ＋ websockets） |
| `clients/h3game/` | 既存ゲーム h3game の bridge.py を h3-control-server へつなぐパッチとバックエンド層（h3game リポジトリには push していない） |

## 安全上の前提

- 電源基板には AC100V が露出している。測定は原則として電源を抜き、残留電圧を確認してから行う。
- 人が乗る装置である。モーター試験は無人・機構を支持・可動端から余裕を取り・物理的な停止手段を用意した状態で行う。
- ブレーキは **無通電で作動、24V 通電で解除**。解除しただけで荷重により機構が動く可能性がある。
