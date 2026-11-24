# xgo_lite2_ros2

四足歩行ロボット **XGO-Lite2** 向けの ROS 2 Jazzy パッケージ集。ベンダー提供の `xgolib.py`（シリアル通信 API）をラップしたもの

## パッケージ構成

| パッケージ | ビルド種別 | 概要 |
| --- | --- | --- |
| [`xgo_lite2_msgs`](xgo_lite2_msgs) | `ament_cmake` | カスタム `msg` / `srv` / `action` 定義（`BodyPosture`, `LegPosition`, `PlayPreset`, `SetGaitType`, `SetPace`, `SetMotorLoad`） |
| [`xgo_lite2_driver`](xgo_lite2_driver) | `ament_python` | `xgolib.py` をラップし、トピック・サービス・アクションを公開する ROS 2 ノード |

## クイックスタート

### 1. 前提条件
- ROS 2 **Jazzy** がセットアップ済みであること
- ロボットのシリアルポートにアクセス可能であること（既定: `/dev/ttyAMA0`）
- Python パッケージ `pyserial` が利用可能であること

```bash
# ユーザーを dialout グループへ追加してシリアルデバイスにアクセス許可（初回のみ）
sudo usermod -aG dialout $USER
# 反映のため一度ログアウト／再ログインする
```

### 2. ビルド
```
# Raspberry Pi CM4 のようなメモリの少ない環境ではメモリ枯渇を避けるため直列ビルドが安全
MAKEFLAGS="-j1" colcon build --parallel-workers 1

source install/setup.bash
```

> **メモ**: Python パッケージ (`xgo_lite2_driver`) を再ビルドする場合、環境によっては
> `--symlink-install` の `--editable` オプションが setuptools に受け付けられません。
> その場合は `rm -rf build/xgo_lite2_driver install/xgo_lite2_driver` してから通常ビルドしてください。

### 3. ノードの起動

```bash
# 既定の xgo_params.yaml で起動
ros2 launch xgo_lite2_driver xgo_lite2_driver.launch.py
```

パラメータを差し替える場合:

```bash
ros2 launch xgo_lite2_driver xgo_lite2_driver.launch.py \
  params_file:=/path/to/xgo_params.yaml
```

### 4. 動作確認（別ターミナル）

```bash
source ~/ros2_ws/install/setup.bash

# トピック一覧を確認
ros2 topic list

# 前進（linear.x=0.1 m/s 相当のスケール指令）
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'

# 停止
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'

# 胴体姿勢（roll=0, pitch=10 deg, yaw=0, 足先固定）
ros2 topic pub -1 /body_posture xgo_lite2_msgs/msg/BodyPosture \
  '{translation: {x: 0.0, y: 0.0, z: 90.0}, attitude: {x: 0.0, y: 10.0, z: 0.0}}'

# プリセットアクション再生（例: 19 = お手 / Handshake）
ros2 action send_goal /play_action xgo_lite2_msgs/action/PlayPreset \
  '{action_id: 19, duration: 3.0}' --feedback

# IMU 自律安定モード ON
ros2 service call /set_imu_mode std_srvs/srv/SetBool '{data: true}'

# 初期状態に戻す
ros2 service call /reset std_srvs/srv/Trigger
```

### 5. 観測値の購読

```bash
ros2 topic echo /joint_states       # 関節角（12 脚サーボ, ラジアン）
ros2 topic echo /imu/data           # 姿勢（クォータニオン）
ros2 topic echo /battery_state      # バッテリー（percentage: 0.0〜1.0）
ros2 topic echo /odom               # cmd_vel 積分によるオープンループ Odometry
```

---

## トピック／サービス／アクション一覧

### 指令系トピック
| インターフェース | 型 | 対応する xgolib API |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | `move_x` / `move_y` / `turn` |
| `/body_posture` | `xgo_lite2_msgs/BodyPosture` | `translation` / `attitude` |
| `/joint_cmds` | `sensor_msgs/JointState`（rad） | `motor()` |
| `/leg_cmd` | `xgo_lite2_msgs/LegPosition`（mm） | `leg()` |
| `/arm_cmd` | `geometry_msgs/Point`（mm; y 無視） | `arm(x, z)` |
| `/claw_cmd` | `std_msgs/UInt8`（0=開, 255=閉） | `claw()` |
| `/mark_time_height` | `std_msgs/UInt8`（mm） | `mark_time()` |
| `/motor_speed` | `std_msgs/UInt8`（1..255） | `motor_speed()` |

### サービス
| サービス | 型 | 対応する xgolib API |
| --- | --- | --- |
| `/set_imu_mode` | `std_srvs/SetBool` | `imu()` |
| `/set_perform_mode` | `std_srvs/SetBool` | `perform()` |
| `/set_arm_mode` | `std_srvs/SetBool` | `arm_mode()` |
| `/set_calibration` | `std_srvs/SetBool` | `calibration()` |
| `/set_gait_type` | `xgo_lite2_msgs/SetGaitType` | `gait_type()`（`trot` / `walk` / `high_walk`） |
| `/set_pace` | `xgo_lite2_msgs/SetPace` | `pace()`（`normal` / `slow` / `high`） |
| `/set_motor_load` | `xgo_lite2_msgs/SetMotorLoad` | `(un)load_motor` |
| `/reset` | `std_srvs/Trigger` | `reset()` |

### アクション
| アクション | 型 | 対応する xgolib API |
| --- | --- | --- |
| `/play_action` | `xgo_lite2_msgs/PlayPreset` | `action(id)`（id 1..255、1..20 が公式プリセット） |

### 観測系トピック
| トピック | 型 | 対応する xgolib API |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState`（rad） | `read_motor()` |
| `/imu/data` | `sensor_msgs/Imu` | `read_roll` / `read_pitch` / `read_yaw` |
| `/battery_state` | `sensor_msgs/BatteryState` | `read_battery()` |
| `/odom` | `nav_msgs/Odometry` | `/cmd_vel` 積分によるオープンループ推定 |

---

## 主要パラメータ

`xgo_lite2_driver/config/xgo_params.yaml` で設定します。

| パラメータ | 既定値 | 説明 |
| --- | --- | --- |
| `serial_port` | `/dev/ttyAMA0` | シリアルポート |
| `baudrate` | `115200` | ボーレート |
| `version` | `auto` | `auto` = `read_firmware()` で自動判定（`M...` → `xgomini`, その他 → `xgolite`） |
| `linear_gain_x` | `25.0` | `cmd_vel.linear.x` [m/s] → xgolib step へのゲイン |
| `linear_gain_y` | `18.0` | `cmd_vel.linear.y` [m/s] → xgolib step へのゲイン |
| `angular_gain` | `57.3` | `cmd_vel.angular.z` [rad/s] → deg/s 変換 |
| `cmd_vel_timeout` | `0.5` | この秒数以上 `cmd_vel` が届かないと自動停止 |
| `joint_names` / `motor_ids` | 12 関節 | URDF に合わせて上書き可 |
| `joint_state_rate_hz` | `20.0` | `/joint_states` 出力周期（0 で無効） |
| `imu_rate_hz` | `20.0` | `/imu/data` 出力周期 |
| `battery_rate_hz` | `1.0` | `/battery_state` 出力周期 |
| `odom_rate_hz` | `20.0` | `/odom` 出力周期 |
| `publish_odom_tf` | `false` | `odom → base_link` TF を発行するか |
| `default_preset_duration` | `3.0` | `PlayPreset` の既定完了待ち時間 [秒] |

---

## 補足事項

- **自動機種判定**: `version: auto` の場合、起動時に `read_firmware()` を呼び出して XGO-Mini/Lite を自動選択します。明示指定したい場合は `xgolite` または `xgomini` を設定してください。
- **シリアル読み出しのブロッキング対策**: `read_*` は最大 ~1 秒ブロックし得るため、`MultiThreadedExecutor` + `ReentrantCallbackGroup` 構成で走らせ、`xgolib` 呼び出しは単一の `threading.Lock` で直列化しています。これにより読み出し遅延がコマンド購読を止めることはありません。
- **オドメトリの制約**: `xgolib` からは脚の運動学ベースの位置フィードバックが取得できないため、`/odom` は `cmd_vel` の積分によるオープンループ推定です。Nav2 等で厳密な自己位置推定が必要な場合は、LiDAR/Visual Odometry など外部センサへ委譲してください。
- **`xgolib.py` の再配布**: 上流ライセンスに従います。詳細はファイル冒頭のヘッダーを参照してください。

## 参考リンク
- [Elecfreaks CM4 XGO Robot Kit — Python 開発ガイド](https://wiki.elecfreaks.com/en/pico/cm4-xgo-robot-kit/advanced-development/python-development/)
- [XGO-PythonLib (公式)](https://github.com/Xgorobot/XGO-PythonLib)

## ライセンス
Apache-2.0（[`LICENSE`](LICENSE) を参照）
