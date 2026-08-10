# scratch_hci_bridge

Scratch 3のHCI extensionからHTTPで受けた絶対角度命令を、OperaSim-PhysX向けのROS 2 topicへ送る`ament_python`パッケージです。rosbridgeを介さず、`rclpy`から直接publishします。

## 接続

```text
Scratch HCI command block
  -> GET http://<ROS_PC>:8899/?type=command&cmd=<cmd>&val=<val>
  -> scratch_hci_bridge
  -> ROS 2 topic
  -> ROS-TCP-Endpoint
  -> OperaSim-PhysX
```

既定設定では`tb20e`を50 Hzで操作します。

| 入力 | ROS 2出力 |
|---|---|
| 左右クローラ速度 | `/tb20e/tracks/cmd_vel` (`geometry_msgs/msg/Twist`) |
| 各関節の目標角度 | `/tb20e/<joint>/cmd` (`std_msgs/msg/Float64`, rad) |

## ビルドと起動

ROS 2 HumbleをセットアップしたUbuntu環境で実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros_tcp_endpoint scratch_hci_bridge
source install/setup.bash
```

先にROS-TCP-Endpointを起動し、OperaSim-PhysXをそのendpointへ接続します。すでに別terminalでendpointを起動している場合、この操作は重複して行いません。

```bash
ros2 launch ros_tcp_endpoint endpoint.py
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
- 左右クローラ速度: `left_crawler_speed`, `right_crawler_speed`

関節命令は`cmd=<関節名>&val=<絶対目標角度[deg]>`です。受理した角度を可動域内へ制限し、次のpublish周期からradへ変換して直接送ります。時間補間や`*_speed`命令は使用しません。左右クローラは次の式で`Twist`へ変換します。

```text
linear.x = (right + left) / 2 * drive_linear_scale
angular.z = (right - left) / 2 * drive_angular_scale
```

## ROS parameter

| parameter | default | 説明 |
|---|---:|---|
| `excavator` | `tb20e` | topicの先頭namespace |
| `publish_hz` | `50.0` | 絶対角度と走行命令の再publish周期[Hz] |
| `http_host` | `0.0.0.0` | HTTP bind先 |
| `http_port` | `8899` | HCI extensionから接続するport |
| `drive_linear_scale` | `1.0` | `linear.x`倍率 |
| `drive_angular_scale` | `1.0` | `angular.z`倍率 |

例:

```bash
ros2 launch scratch_hci_bridge scratch_hci_bridge.launch.py \
  excavator:=tb20e http_host:=127.0.0.1 http_port:=8899 publish_hz:=50.0
```

このノードは関節の目標topicと走行topicを継続publishします。同じtopic、または同じ機体を操作する`tb20e_control`などの別制御ノードから同時に命令しないでください。終了時には走行停止の`Twist`を1回publishします。

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
