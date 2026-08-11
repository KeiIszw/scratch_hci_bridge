# scratch_hci_bridge

Scratch 3のHCI extensionからHTTPで受けた絶対角度命令を、TB20eの
`joint_trajectory_controller`へ送る`ament_python`パッケージです。rosbridgeを介さず、
`rclpy`から直接送信します。

## 接続

```text
Scratch HCI command block
  -> GET http://<ROS_PC>:8899/?type=command&cmd=<cmd>&val=<val>
  -> scratch_hci_bridge
  -> /tb20e_controller/follow_joint_trajectory
  -> tb20e_control (PID)
  -> /manipulated_<joint>_lever
  -> Unity
```

既定設定では`tb20e`を50 Hzで操作します。

| 入力 | ROS 2出力 |
|---|---|
| 左右クローラ速度 | `/tb20e/tracks/cmd_vel` (`geometry_msgs/msg/Twist`) |
| `swing`, `boom`, `arm`, `bucket`の目標角度 | `/tb20e_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`, rad) |
| `blade`, `body`, `thumb`の目標角度 | `/tb20e/<joint>/cmd` (`std_msgs/msg/Float64`, rad) |

## ビルドと起動

ROS 2 HumbleをセットアップしたUbuntu環境で実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb20e_control scratch_hci_bridge
source install/setup.bash
```

先にUnityのTB20eシーンとROS接続を起動し、別terminalで`tb20e_control`を起動します。

```bash
ros2 launch tb20e_control tb20e_control.launch.py
```

別terminalで本ノードを起動します。

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch scratch_hci_bridge scratch_hci_bridge.launch.py
```

ScratchとROS 2が同じPCなら、HCI extensionの既定値`http://localhost`, port `8899`をそのまま使えます。別PCなら、HCIの`set url [baseurl] port [port]`ブロックを命令ブロックより先に実行し、`baseurl`を`http://<ROS_PCのIP>`、portを`8899`に設定します。`baseurl`にはschemeを含め、portや末尾の`/`は含めません。`set url`はScratch内の送信先を変えるブロックであり、このROS 2ノードへ設定要求を送るものではありません。

HTTPだけを確認する例:

```bash
curl 'http://localhost:8899/?type=command&cmd=boom&val=-30'
curl 'http://localhost:8899/?type=command&cmd=bucket&val=45'
curl 'http://localhost:8899/?type=command&cmd=left_crawler_speed&val=0.5'
curl 'http://localhost:8899/?type=command&cmd=right_crawler_speed&val=0.5'
```

成功時は`200 OK`と本文`OK`、入力不正時は`400`を返します。ただしHCIのcommand blockは`no-cors`で応答を待たないため、Scratch画面からstatusや本文は確認できません。ノードログには受理した命令が表示されます。不正命令の理由は同じURLを`curl`で実行して確認してください。

同じ理由で、連続するHTTP命令の呼出順は通信上保証されません。本ノードは到着した順に処理します。最終状態が順序に依存する命令間にはScratchの`待つ`ブロックを入れてください。厳密な順序保証が必要な場合は、HCI extension側の`commandURL`が`fetch`のPromiseを返すよう変更する必要があります。

## command

- 絶対目標角度[deg]: `blade`, `body`, `swing`, `boom`, `arm`, `bucket`, `thumb`
- PID関節の角速度上限[deg/s]: `swing_speed`, `boom_speed`, `arm_speed`,
  `bucket_speed`
- 左右クローラ速度: `left_crawler_speed`, `right_crawler_speed`

関節命令は`cmd=<関節名>&val=<絶対目標角度[deg]>`です。`swing`, `boom`,
`arm`, `bucket`は、受理した角度を可動域内へ制限し、4軸をまとめてradへ変換して
`FollowJointTrajectory` goalとして送ります。今回のHTTPバッチで指定されていない軸は、
送信時点の最新`/joint_states`角度で保持します。現在角度が揃う前に受信した命令は破棄せず
保留します。送信済みの目標は次のgoalへ引き継がないため、未達または失敗した軸が後続する
別軸の命令を妨げません。

短時間に複数軸のHTTP命令を受信した場合は、`trajectory_debounce_sec`の間待って1つのgoalへ
まとめます。同じgoalを50 Hzで再送しません。角速度上限を設定していない軸の到達時間は
`trajectory_duration_sec`で指定します。PID関節goalは位置のみの線形補間を使います。

PID関節は`cmd=<関節名>_speed&val=<角速度上限[deg/s]>`を先に送ると、その後の角度goalに
角速度上限を適用します。到達時間は次のように自動計算されます。

```text
角速度上限設定軸の到達時間 = |目標角度 - 現在角度| / 角速度上限
```

角速度上限設定時に`trajectory_duration_sec`を最短時間としては使用しません。複数軸を同時に
変更した場合は、各軸について上式で求めた時間の最大値をgoal全体へ適用します。角速度上限が
設定されていない変更軸が含まれる場合、その軸には従来どおり`trajectory_duration_sec`を使います。

複数軸を同じHTTPバッチで更新した場合は、全軸の速度上限を守れる共通到達時間を使います。
`0`を送ると、その軸の速度上限を解除して固定到達時間方式へ戻します。負値と非数は拒否します。

```bash
# boomを最大5 deg/sに設定し、その後-30 degreeへ動かす
curl 'http://localhost:8899/?type=command&cmd=boom_speed&val=5'
curl 'http://localhost:8899/?type=command&cmd=boom&val=-30'

# boomの角速度上限を解除
curl 'http://localhost:8899/?type=command&cmd=boom_speed&val=0'
```

速度値は軌道指令上の角速度上限です。実際のfeedback速度は角度計測ノイズ、速度推定、
PID追従のオーバーシュートにより一時的に上限を超える場合があります。この機能は実機向けの
独立した安全速度リミッタの代わりにはなりません。

`blade`, `body`, `thumb`はtb20eのPID対象外であり、従来どおり個別topicへ直接送ります。
左右クローラは次の式で`Twist`へ変換します。

```text
linear.x = (right + left) / 2 * drive_linear_scale
angular.z = (right - left) / 2 * drive_angular_scale
```

## ROS parameter

| parameter | default | 説明 |
|---|---:|---|
| `excavator` | `tb20e` | topicの先頭namespace |
| `publish_hz` | `50.0` | 走行・PID対象外関節のpublish周期とgoal送信確認周期[Hz] |
| `http_host` | `0.0.0.0` | HTTP bind先 |
| `http_port` | `8899` | HCI extensionから接続するport |
| `drive_linear_scale` | `1.0` | `linear.x`倍率 |
| `drive_angular_scale` | `1.0` | `angular.z`倍率 |
| `trajectory_action` | `/tb20e_controller/follow_joint_trajectory` | 4軸目標の送信先Action |
| `joint_state_topic` | `/joint_states` | 未指定軸の初期化に使う現在角度 |
| `trajectory_duration_sec` | `3.0` | 角速度上限を設定していない軸の到達時間[s] |
| `trajectory_debounce_sec` | `0.05` | 連続する角度命令をまとめる待ち時間[s] |

例:

```bash
ros2 launch scratch_hci_bridge scratch_hci_bridge.launch.py \
  excavator:=tb20e http_host:=127.0.0.1 http_port:=8899 publish_hz:=50.0
```

HTTPの`200 OK`は命令を内部状態へ保存したことを示し、trajectory goalの受理や完了までは
保証しません。goalの受理・失敗はノードログへ出力します。PID出力topicへ別ノードから同時に
指令しないでください。終了時には走行停止の`Twist`を1回publishします。

`http_host=0.0.0.0`ではLAN上の全interfaceから命令を受けます。Scratchと同じPCだけで使う場合は`http_host:=127.0.0.1`へ制限してください。このHTTP APIに認証機能はありません。

## テスト

```bash
cd ~/ros2_ws/src/scratch_hci_bridge
PYTHONPATH=. python3 -m unittest discover -s test -v
```

ROS 2パッケージとして検証する場合:

```bash
cd ~/ros2_ws
colcon test --packages-select scratch_hci_bridge --event-handlers console_direct+
colcon test-result --verbose
```
