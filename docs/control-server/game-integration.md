# ゲーム側の組み込みガイド（h3game → h3-control-server）

ゲーム（h3game）の実装者向けの入口。座席（DOF Reality H3 改造機）は PC 上の **h3-control-server**（Rust）が 3 軸の CoreS3 を USB で駆動し、ゲームはサーバーに姿勢（pitch / roll / yaw / gain）を送るだけ。実機が無くても模擬軸（sim）で同じ API を試せる。

| 知りたいこと | 読む場所 |
| --- | --- |
| サーバーの起動・設定・管理操作（curl） | [README.md](README.md) |
| Game API の契約（WS `/v1/motion` の JSON、ack の拒否理由、`GET /v1/status` の形） | [../../control-server/DESIGN.md](../../control-server/DESIGN.md) §7 |
| h3game に当てるパッチと bridge の動かし方 | [../../clients/h3game/README.md](../../clients/h3game/README.md) |
| Python クライアント（bridge が使う。他のゲームからも使える） | [../../clients/python/README.md](../../clients/python/README.md) |
| 元の設計（なぜこうなっているか） | [architecture.md](architecture.md) §2〜§4 |

## 1. 実機なしで動かす（最短）

```sh
# 1) サーバー（模擬軸。トークン不要）
cd control-server && cargo build && ./target/debug/h3-control-server --config config/sim.toml
# 2) 運転用テストページ（状態・ARM/START/STOP・姿勢スライダー）
open http://127.0.0.1:8892/
# 3) h3game（パッチ適用後）を server モードで
H3_BACKEND=server python3 bridge.py        # Chrome で http://localhost:8891
```

模擬軸は `config/sim.toml`（`calibration_valid = true` は模擬軸だから許される）。実機用の設定は `config/bringup.toml` と [hardware-bringup.md](hardware-bringup.md)。

自動の結合試験: `control-server/scripts/e2e_sim.sh`（サーバー＋パッチ後 bridge＋ヘッドレス Chromium、120 項目）。

## 2. ゲームがやること・やらないこと

- **やること**: `POST /v1/session` で session を取り、`WS /v1/motion?session=...` に **`frame_seq` が進む姿勢**を送り続ける（DESIGN.md §7.1）。各フィールド:
  - `pitch` / `roll` / `yaw`: −1..1（範囲外は互換モードでクリップされ `limits` に出る）
  - `gain`: 0..1 の要求値。サーバーの `max_gain`（`[limits]` の既定 0.5。bring-up 中は 0.15 に下げてある）で頭打ちになり、ack の `effective_gain` で分かる
- **動きの制限を変えたいとき**（[limits.md](limits.md) §3.1）: 同じ WS に `{"type":"limits","protocol":1,"source_session":"...","seq":N,"max_gain":0.8,"max_vel_count_s":3000,"max_acc_count_s2":15000,"duty_max_percent":40}`（どの項目も省略可）。サーバーはファームの絶対上限で頭打ちにし、ack の `effective_limits` と `limits`（`ceiling_clamped`）で返す。鮮度は延びない。`POST /v1/session` のたびに設定ファイルの初期値に戻るので、接続のたびに送り直す。bridge は POST の `"limits": {...}` をそのまま渡す
  - `seq`: 単調増加（接続ごと）。`frame_seq`: **ゲームのフレームが進んだときだけ増やす**。同じ値の再送では鮮度が延びない
  - `source_age_ms`: 入力の古さ（0 でよい）
- **やらないこと**: ARM / START / STOP / 校正は**操作者**（テストページまたは curl）。ゲームは駆動権限を持たない。ゲームがつながっただけでは座席は動かない。
- **止まるとき**: `frame_seq` が `source_freshness_ms`（既定 250 ms）進まない、WS が切れる、session が変わる（再接続・再読み込み）、`/v1/stop`。いずれも減速 → 保持 → ブレーキ（中央へは戻さない）。再開は操作者の ARM ＋ START。
- **ack の拒否理由**（DESIGN.md §7.1）: `stale_session` / `old_seq` / `old_frame` / `non_finite` / `out_of_range` / `invalid_age` / `missing_frame_seq` / `bad_protocol` / `bad_message` / `axis_conflict`。拒否されたフレームは捨てられる（止めはしない）。

混合式は旧 h3game と同じ（`m1 = 512 + g(roll + pitch)`, `m2 = 512 + g(roll − pitch)`, `m3 = 512 + g·yaw`、architecture.md §2）。軸の向きの違いはサーバー側の校正で吸収するので、ゲーム側で符号を変えない。

## 3. 画面に出すとよい状態（`GET /v1/status`、WS の `status`）

- `rig_state`: `OFFLINE` / `CALIBRATION_REQUIRED` / `READY_TO_ARM` / `ARMING` / `HOLDING` / `ACTIVE` / `STOPPING` / `BRAKED_DISARMED` / `FAULT_LATCHED`
- `blockers`（`"<code>: <detail>"`。`:` より前の code で判定）、`fault`、`axis_conflict`
- `session.fresh`（ゲーム入力が新鮮か）、`effective_gain`、`limits`、`motion_limits`（いまの制限値と上限）
- 軸ごと: `mcu_state` / `fault_code` / `actual_count` / `following_error_count` など

bridge のパッチは HUD に `rig_state`・`blockers`・`hud_note`（系統の競合 / 個体の不一致 / 再基準化が必要）を出す（clients/h3game/README.md）。

## 4. 相談なく変えないこと

- WS の JSON のフィールド名・`protocol = 1`・拒否理由の文字列（サーバー側の試験とクライアントが依存）。
- `frame_seq` の意味（進んだときだけ鮮度を延ばす）。これが安全の要。
- bind は loopback 専用（`server.allow_non_loopback = false`）。LAN からは `GET /v1/status` と `POST /v1/stop` しか通らない。

質問・変更要望は Issue へ。プロトコルの変更は `control-server/DESIGN.md` §7 と `clients/` の試験を同時に直す。
