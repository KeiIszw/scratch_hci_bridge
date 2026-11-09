# scratch_hci_bridge

Scratch 3のHCI extensionからHTTPで受けた絶対・相対角度命令を、TB20eの
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
  -> 実機
```

既定設定では`tb20e`を50 Hzで操作します。

| 入力 | ROS 2出力 |
|---|---|
| 左右クローラ速度 | `/tb20e/tracks/cmd_vel` (`geometry_msgs/msg/Twist`) |
| `swing`, `boom`, `arm`, `bucket`の絶対・相対角 | `/tb20e_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`, rad) |
| `blade`, `body`, `thumb`の目標角度 | `/TB20e/<joint>/cmd` (`std_msgs/msg/Float64`, rad) |

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
# 既定はserial。明示する場合もvalにはダミー値を指定する
curl 'http://localhost:8899/?type=command&cmd=serial&val=1'
curl 'http://localhost:8899/?type=command&cmd=boom_angle&val=-30'
curl 'http://localhost:8899/?type=command&cmd=boom_up&val=10'
curl 'http://localhost:8899/?type=command&cmd=swing_left&val=30'
curl 'http://localhost:8899/?type=command&cmd=bucket_dump&val=15'

# 以後のPID関節命令をparallelへ切り替える
curl 'http://localhost:8899/?type=command&cmd=parallel&val=1'
curl 'http://localhost:8899/?type=command&cmd=boom_angle&val=-50'
curl 'http://localhost:8899/?type=command&cmd=arm_angle&val=100'

# crawlerは実行モードの対象外
curl 'http://localhost:8899/?type=command&cmd=left_crawler_speed&val=0.5'
curl 'http://localhost:8899/?type=command&cmd=right_crawler_speed&val=0.5'
```

成功時は`200 OK`と本文`OK`、入力不正時は`400`を返します。HCIのcommand blockは
`no-cors`のためstatusや本文をScratch画面から確認できませんが、同梱版は通信完了を待って
次のブロックへ進みます。ノードログには受理した命令が表示されます。不正命令の理由は同じ
URLを`curl`で実行して確認してください。

同梱のHCI extensionは`commandURL`から`fetch`のPromiseを返し、HTTP受付が完了してから
次のcommand blockへ進むため、ブロックの呼出順を維持します。古いHCI extensionは連続する
`fetch`の完了を待たないため更新してください。本ノード側でもHTTPサーバへ到着した順を保持し、
古いextensionで最初のモード命令だけがわずかに遅着した場合は直前の未確定バッチへ適用します。

## command

- 絶対目標角度[deg]: `blade_angle`, `body_angle`, `swing_angle`, `boom_angle`, `arm_angle`, `bucket_angle`, `thumb_angle`
- PID関節の相対移動量[deg]: `swing_left`, `swing_right`, `boom_up`,
  `boom_down`, `arm_extend`, `arm_retract`, `bucket_dump`, `bucket_scoop`
- PID関節の角速度上限[deg/s]: `swing_speed`, `boom_speed`, `arm_speed`,
  `bucket_speed`
- PID関節の実行モード: `serial`, `parallel`（`val`は未使用）
- 左右クローラ速度: `left_crawler_speed`, `right_crawler_speed`

旧形式の `boom` など軸名だけのコマンドは受け付けません。

絶対関節命令は`cmd=<関節名>_angle&val=<絶対目標角度[deg]>`です。`swing`, `boom`,
`arm`, `bucket`は、受理した角度を可動域内へ制限し、4軸をまとめてradへ変換して
`FollowJointTrajectory` goalとして送ります。今回のHTTPバッチで指定されていない軸は、
直前に成功したゴールの目標角度で保持します。ただし実測との誤差が
`relative_basis_max_error_deg`を超える軸、またはstop/reset/失敗の後は最新の実測角度を使います。
現在角度が揃う前に受信した命令は破棄せず保留し、実測値は成功目標で上書きしません。

相対関節命令はOCS20250118と同じく、`val`の符号を無視して移動量の大きさ[deg]として
使用します。方向はコマンド名で決まります。

| HTTP command | 対象軸 | 目標角度 |
|---|---|---|
| `swing_left` / `swing_right` | swing（車体上部旋回） | 現在角 `+ abs(val)` / `- abs(val)` |
| `boom_up` / `boom_down` | boom | 現在角 `- abs(val)` / `+ abs(val)` |
| `arm_extend` / `arm_retract` | arm | 現在角 `- abs(val)` / `+ abs(val)` |
| `bucket_dump` / `bucket_scoop` | bucket | 現在角 `- abs(val)` / `+ abs(val)` |

```bash
# 現在のboom角度から10 degree上げる（val=-10でも移動量は10 degree）
curl 'http://localhost:8899/?type=command&cmd=boom_up&val=-10'
```

相対角はHTTP受信時ではなく、その命令を実行する直前のfeedbackから計算します。
`boom`, `arm`, `bucket`は絶対・相対命令を合成した後に可動域へ制限します。
これら3軸ではOperaSimの関節座標に合わせ、up/extend/dumpを負方向、
down/retract/scoopを正方向として扱います。
`swing_left`はROS正方向、`swing_right`はROS負方向へ車体上部を連続旋回させます。
方向を保証するため、1命令の移動量は`abs(val) < 180` degreeに制限し、180 degree以上は
`400 Bad Request`で拒否します。例えば現在角170 degreeで`swing_left=30`を実行すると、
目標をラップせず200 degreeとして送り、右へ160 degree逆回転することを防ぎます。

### serial / parallel

起動時の実行モードは`serial`です。モードの変更は次のHTTP命令で行います。共通の
command block形式に合わせて`val`が必要ですが、モード選択には値を使用しません。

```bash
curl 'http://localhost:8899/?type=command&cmd=serial&val=1'
curl 'http://localhost:8899/?type=command&cmd=parallel&val=1'
```

`serial`ではバッチ内のPID関節命令をHTTP到着順のFIFOで1件ずつgoalにし、先行goalが
`FollowJointTrajectory` Actionで成功完了してから次のgoalを送ります。例えば次の命令では、
boomを現在角から10 degree上げ終えてから、armをその時点の角度から15 degree縮めます。

各goalには既定で位置許容差0.75 degree、停止速度許容値0.5 degree/sを明示し、
controllerがその条件で成功を返すまで後続を開始しません。

```bash
curl 'http://localhost:8899/?type=command&cmd=serial&val=1'
curl 'http://localhost:8899/?type=command&cmd=boom_up&val=10'
curl 'http://localhost:8899/?type=command&cmd=arm_retract&val=15'
```

TB20eのarm可動域は32--155 degreeです。絶対命令`arm=30`や、相対命令を合成した結果が
可動域外になる場合は、従来どおり32 degreeへ制限されます。

`parallel`では`trajectory_debounce_sec`内に連続して届いたPID関節命令を1つの4軸goalへ
まとめ、各軸を同時に開始します。同じバッチで同じ軸を複数回指定した場合は、最後の絶対角を
使用します。相対命令は共通の基準角を原点として同じ軸の移動量を加算します。絶対・相対が
混在した軸は「最後の絶対角 + 全相対移動量」を目標にします。厳密な命令順で同じ軸を段階的に
動かす場合は`serial`を使用してください。実行中のgoalがある場合は、モードにかかわらず
後続バッチをFIFOで待機させ、先行goalを新しいgoalで途中終了させません。
連続関節で左右方向を保証するため、同じparallelバッチに複数の`swing`系命令がある場合だけは
到着順の個別goalへ自動分割します。`swing`の絶対角と`swing_left/right`の混在も同様です。

```bash
# 同じfeedback snapshotからboomとarmを同時に相対移動する
curl 'http://localhost:8899/?type=command&cmd=parallel&val=1'
curl 'http://localhost:8899/?type=command&cmd=boom_up&val=10'
curl 'http://localhost:8899/?type=command&cmd=arm_retract&val=15'
```

モード命令はバッチ境界です。すでに確定済みまたは実行中のバッチのモードは変更せず、
後続命令へ新しいモードを適用します。例外として、起動後最初の明示的なモード命令が
未確定の運動命令よりわずかに遅着した場合だけ、その未確定バッチへ最初のモードを適用します。
このためモード未指定時の最初のバッチは`initial_mode_command_wait_sec`だけ待ち、既定の
`serial`で実行します。最初からモード命令を送った場合、通常の
`trajectory_debounce_sec`だけ待ちます。

Actionが拒否・中断・キャンセル・失敗した場合、serialの後続だけが動くことを防ぐため、
待機中のPID関節命令を破棄します。選択中のモードは維持されるので、その後に受信した新しい
関節命令は実行できます。

goal応答が`trajectory_goal_response_timeout_sec`を超えて返らない場合は、サーバ側で受理済みか
判定できないため、安全側として実行キューを閉じたままエラーを記録します。受理済みgoalが
予定時間と`trajectory_result_timeout_margin_sec`の合計を超えても終端しない場合はcancelを要求し、
cancel後の終端resultを確認するまで後続goalを送りません。Action通信障害により終端状態を確認
できない場合は、重複goalを防ぐためノードを再起動するまでキューを閉じたままにします。

角速度上限を設定していない軸の到達時間は`trajectory_duration_sec`で指定します。
PID関節goalは位置のみの線形補間を使い、同じgoalを50 Hzで再送しません。

PID関節は`cmd=<関節名>_speed&val=<角速度上限[deg/s]>`を先に送ると、その後に受信した
角度命令へ角速度上限を記録します。待機中に速度設定を変更しても、受信済み命令の速度は
変わりません。到達時間は次のように自動計算されます。

```text
角速度上限設定軸の到達時間 = |制限後の目標角度 - 実行開始時の現在角度| / 角速度上限
```

角速度上限設定時に`trajectory_duration_sec`を最短時間としては使用しません。parallelで複数軸を同時に
変更した場合は、各軸について上式で求めた時間の最大値をgoal全体へ適用します。角速度上限が
設定されていない変更軸が含まれる場合、その軸には従来どおり`trajectory_duration_sec`を使います。

複数軸を同じHTTPバッチで更新した場合は、全軸の速度上限を守れる共通到達時間を使います。
parallelの同じ軸へ複数の命令が寄与する場合は、記録された角速度上限のうち最も遅い値を使い、
上限未設定の命令も含まれる場合は`trajectory_duration_sec`も下限候補にします。
`0`を送ると、その軸の速度上限を解除して固定到達時間方式へ戻します。負値、非数、および
ROSの`Duration`で表現できないほど小さい正値は拒否します。

```bash
# boomを最大5 deg/sに設定し、その後-30 degreeへ動かす
curl 'http://localhost:8899/?type=command&cmd=boom_speed&val=5'
curl 'http://localhost:8899/?type=command&cmd=boom_angle&val=-30'

# boomの角速度上限を解除
curl 'http://localhost:8899/?type=command&cmd=boom_speed&val=0'
```

速度値は軌道指令上の角速度上限です。実際のfeedback速度は角度計測ノイズ、速度推定、
PID追従のオーバーシュートにより一時的に上限を超える場合があります。この機能は実機向けの
独立した安全速度リミッタの代わりにはなりません。

実行モードの対象はAction完了を判定できる`swing`, `boom`, `arm`, `bucket`です。
`blade`, `body`, `thumb`はtb20eのPID対象外であり、従来どおり個別topicへ直接送ります。
そのためOCSの`base_left/right`, `blade_up/down`, `thumb_open/close`とcrawlerの方向・角度
コマンドは未対応です。OperaSimでは車体上部旋回を`swing_left/right`として公開します。
OCSの`base`は車体上部旋回、OCSの`swing`は別のブームスイング関節なので、`base_left/right`
を同じ軸の別名にはしていません。今回OCSを参考に追加した相対コマンドは上記の
`swing`, `boom`, `arm`, `bucket`用コマンドであり、
`boom_angle`などの絶対角別名や、大文字・hyphen・空白を含むコマンド名の正規化は行いません。
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
| `trajectory_debounce_sec` | `0.5` | 連続する角度命令をまとめる待ち時間[s] |
| `initial_mode_command_wait_sec` | `0.5` | 最初の明示モードが遅着しても未確定バッチへ適用できる待ち時間[s] |
| `joint_state_timeout_sec` | `0.5` | 相対角と未指定軸の基準に使えるfeedbackの最大経過時間[s]（`0`で監視無効） |
| `trajectory_goal_tolerance_deg` | `0.75` | Action成功とみなす各PID関節の位置許容差[deg] |
| `trajectory_stopped_velocity_deg_s` | `0.5` | Action成功とみなす各PID関節の停止速度許容値[deg/s] |
| `trajectory_goal_response_timeout_sec` | `5.0` | goal応答待ちの警告時間[s]（超過時は状態不明のためキューを閉じたまま保持） |
| `trajectory_result_timeout_margin_sec` | `35.0` | 指定到達時間を超えてresultを待つ猶予[s]（超過後はcancel要求） |
| `trajectory_shutdown_cancel_timeout_sec` | `2.0` | ノード終了時にgoal応答・cancel・終端resultを待つ合計上限[s] |
| `unity_position_output_enabled` | `false` | 受理された4軸目標をUnity用topicにも送る |
| `unity_position_command_prefix` | `/TB20e` | Unity用位置指令topicのprefix（`/<axis>/cmd`を付加） |
| `direct_joint_command_prefix` | `/TB20e` | Unity用`blade/body/thumb` topicのprefix |
| `swing_unity_position_sign` | `-1.0` | Unity用swing位置指令の符号（実機座標との変換） |
| `boom_unity_position_sign` | `1.0` | Unity用boom位置指令の符号 |
| `arm_unity_position_sign` | `1.0` | Unity用arm位置指令の符号 |
| `bucket_unity_position_sign` | `1.0` | Unity用bucket位置指令の符号 |

例:

```bash
ros2 launch scratch_hci_bridge scratch_hci_bridge.launch.py \
  excavator:=tb20e http_host:=127.0.0.1 http_port:=8899 publish_hz:=50.0
```

`unity_position_output_enabled:=true`の場合、実機側Action serverがgoalを受理した後に、
同じ目標角度を`/TB20e/{swing,boom,arm,bucket}/cmd`へ
`std_msgs/msg/Float64`のradでpublishします。実機とUnityをまとめて起動する場合は
`tb20e_bringup`の`tb20e_real_unity.launch.py`を使用してください。
topic prefixは先頭に`/`がない指定でも絶対名へ正規化されます。swingはUnityと実機の
正方向を合わせるため、既定で目標角度の符号を反転します。

HTTPの`200 OK`は命令を内部キューへ保存したことを示し、trajectory goalの受理や完了までは
保証しません。goalの送信・受理・完了・失敗と実行モードはノードログへ出力します。PID出力topicへ別ノードから同時に
指令しないでください。終了時には走行停止の`Twist`を1回publishし、実行中のtrajectoryへ
cancelを要求して`trajectory_shutdown_cancel_timeout_sec`の範囲で応答を待ちます。

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

## Measured motion reporter

GET /?type=reporter&cmd=motion_status&val=0 returns idle, busy, or failed.
Busy includes batching, queued and running goals. Failure is latched until node restart.
This read-only endpoint is used by tb20e_motion_capture between measured segments.
Scratch timeout stops subsequent commands but does not cancel an active Action.


## Scratch同期と停止

既存の `parallel` / `serial`、`*_angle`、`*_speed`、相対指令は引き続き使用できます。
新しいプログラムでは、debounceで並列区間が分割されないよう明示バッチを使用します。
以下はHTTP GETの `type` / `cmd` / `val` です。

| type | cmd | val | 動作 |
|---|---|---|---|
| command | begin | 0 | parallelの明示バッチを開く |
| command | commit | 0 | 位置許容だけで完了（指令関節のvelocity許容は-1） |
| command | commit | 1 | 位置許容と停止速度許容で完了 |
| command | stop | 任意の有限数値 | 構築中・送信待ちを破棄し、走行を0、軸マスクを0、goalをcancel |
| command | reset | 0 | stop相当の処理を行い、失敗・停止状態と理由を解除 |
| reporter | heartbeat | 0 | `ok` を返し、リース監視を有効にする |
| reporter | motion_status | 0 | `idle` / `busy` / `failed` / `stopped` |
| reporter | motion_error | 0 | 最後の失敗理由（なければ空文字） |

明示バッチ内の `parallel` / `serial` は無視します。空バッチのcommitは何も送りません。
開いたまま再びbeginした場合は、その未確定バッチを置き換えます。
同一バッチに複数の旋回指令がある場合の安全な分割はcommit時に行います。
従来の暗黙バッチは引き続きdebounceまたはモード変更で確定します。

`busy` は構築中・送信待ち・実行中、およびcancelの終端result待ちを含みます。
stopのHTTP応答はcancel完了を待ちません。通常のstop後は終端resultを受けて `stopped` になり、
次の動作指令で解除します。resetも実行中goalの終端を待つ間は `busy` です。
`failed` はresetまで保持します。理由は `lease_expired` / `stale` / `goal_rejected` /
`goal_aborted` / `tolerance_violated` / `result_timeout` / `stopped` / 空文字です。
既存互換のためgoal位置許容違反では既存の待ち列を継続しますが、失敗状態は保持します。

heartbeat受信前はリース監視を行いません。一度有効になると、全HTTP GET要求
（heartbeatやmotion_status、エラー応答になる要求も含む）がリースを更新します。
バッチ構築中・送信待ち・goal実行中に通信が途絶するとstop相当の処理を行い、
理由 `lease_expired` の `failed` にします。
送信期限切れと明示バッチの開放期限切れは理由 `stale` です。
serialの2番目以降の指令は、直前の指令の終端から送信期限を数えます。

| パラメータ | 既定値 | 意味 |
|---|---|---|
| client_lease_sec | 2.0 | heartbeatで有効化後の通信途絶許容秒数 |
| max_dispatch_delay_sec | 1.0 | バッチ確定後の送信待ち上限秒数 |
| batch_open_timeout_sec | 5.0 | beginからcommitまでの上限秒数 |
| relative_basis_max_error_deg | 5.0 | 成功目標を相対基準に使える実測との差 |

追加パラメータは正の有限数値で、同名のlaunch引数から設定できます。
相対指令は直前に成功した指令関節の目標角を基準とし、実測との差が上限を超える場合や
stop/reset/失敗後は実測角に戻します。実測JointState自体は目標値で上書きしません。
HTTPはdaemonスレッドの `ThreadingHTTPServer` です。通常指令は接続受付順に適用し、
stop/resetとreporterはその待ち列を通りません。stop/reset以前から待機していたHTTP指令も
破棄し、停止後の遅延登録を防ぎます。ROSとHTTPの短い状態更新は同一ロックで保護し、
cancel完了やネットワーク応答をそのロック内で待ちません。

### 送信待ちの確認と非指令軸の保持

`/?type=reporter&cmd=motion_queue&val=0` は送信待ちのバッチ数を整数文字列で返します。
構築中（begin直後を含む）と確定済みの未送信バッチを数え、実行中のゴールは除きます。
serialバッチに未送信の指令が残る場合は、そのバッチを1件として数えます。
`motion_status`も併用し、failed/stopped/errorを確認してください。
commit 0の後続バッチは、前のゴールの終端後、次のタイマー周期で送信対象になります。
非指令軸も相対指令と同じ基準で保持します。直前に成功したゴールの目標角を使い、
実測との誤差が`relative_basis_max_error_deg`を超える軸、またはstop/reset/失敗の後は実測角を使います。
