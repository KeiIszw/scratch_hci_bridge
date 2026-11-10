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
直前に指令したゴールの目標角度で保持します。相対指令も同じ目標角度を基準に加算します。
実測との誤差の大きさや途中区間の非致命的な許容違反では基準を変えず、未到達分を次の目標へ含めます。
stop/reset/致命的失敗の後と、まだ目標を送っていない軸は最新の実測角度を使います。
`relative_basis_max_error_deg`はNode・launchとも廃止しました。既存の設定から削除してください。
現在角度が揃う前に受信した命令は破棄せず保留し、実測値は指令目標で上書きしません。

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
動かす場合は`serial`を使用してください。後続バッチはFIFOで待機させます。
明示的な`commit 0`のgoalだけは、確定済みの後続があれば計画時間の経過時に置き換えます。
`commit 1`と暗黙バッチは、従来どおり先行goalの結果を待ちます。
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
待機中のPID関節命令を破棄します。例外として、明示バッチの`commit 0`で
`GOAL_TOLERANCE_VIOLATED`になった場合は、既定で警告だけを出して後続を継続します。
また、意図したgoal置き換えで届く旧goalの結果は失敗にしません。選択中のモードは維持されるので、その後に受信した新しい
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
| `active_axis_mask_topic` | 空文字 | hardwareへ許可軸のbitmaskを送るtopic。空なら無効 |
| `catch_up_axes_in_mask` | `true` | 指令軸に保持目標へ未到達の軸を加えて追従を継続する |
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
Busy includes batching, queued and running goals. Failure is latched until reset.
This read-only endpoint is used by tb20e_motion_capture between measured segments.
The generated Scratch timeout sends stop and requests cancellation of the active Action.


## Scratch同期と停止

既存の `parallel` / `serial`、`*_angle`、`*_speed`、相対指令は引き続き使用できます。
新しいプログラムでは、debounceで並列区間が分割されないよう明示バッチを使用します。
以下はHTTP GETの `type` / `cmd` / `val` です。

| type | cmd | val | 動作 |
|---|---|---|---|
| command | begin | 0 | parallelの明示バッチを開く |
| command | commit | 0 | 確定済み後続があれば計画時間で置換。後続なしは位置許容だけで完了待ち（velocity許容は-1） |
| command | commit | 1 | 位置許容と停止速度許容で完了 |
| command | stop | 任意の有限数値 | 構築中・送信待ちを破棄し、走行を0、軸マスクを0、goalをcancel |
| command | reset | 0 | stop相当の処理を行い、失敗・停止状態と理由を解除 |
| reporter | heartbeat | 0 | `ok` を返し、リース監視を有効にする |
| reporter | motion_status | 0 | `idle` / `busy` / `failed` / `stopped` |
| reporter | motion_error | 0 | reset以降の最初の失敗理由（失敗なしのstopはstopped、理由なしは空文字） |

明示バッチ内の `parallel` / `serial` は無視します。空バッチのcommitは何も送りません。
開いたまま再びbeginした場合は、その未確定バッチを置き換えます。
同一バッチに複数の旋回指令がある場合の安全な分割はcommit時に行います。
従来の暗黙バッチは引き続きdebounceまたはモード変更で確定します。

`busy` は構築中・送信待ち・実行中、およびcancelの終端result待ちを含みます。
stopのHTTP応答はcancel完了を待ちません。通常のstop後は終端resultを受けて `stopped` になり、
次の動作指令で解除します。resetも実行中goalの終端を待つ間は `busy` です。
`failed` はresetまで保持します。理由は `lease_expired` / `stale` / `goal_rejected` /
`goal_aborted` / `tolerance_violated` / `result_timeout` / `stopped` / 空文字です。
失敗検知後にScratchからstopが届いても、`motion_error`と`failed`は最初の原因を保持します。
stopだけで止めた場合は`stopped`、resetで理由を消去します。

`commit 0`の`GOAL_TOLERANCE_VIOLATED`（Action status=ABORTED、code=-5）は、
既定では`failed`にせず待ち列を続行します。`commit 1`、PATH許容違反、その他のABORTED、
REJECTEDは失敗として待ち列を破棄します。
`intermediate_tolerance_failure_fatal:=true`で途中ゴールの位置許容違反も失敗にできます。
`commit 0`のgoalにだけ`intermediate_goal_time_tolerance_sec`（既定0.5秒）を設定し、
軌道の予定終端後に位置許容差へ収まるまでの猶予を短くします。fatal=trueでも猶予は同じで、
違反結果を警告にするか失敗にするかだけが変わります。
`commit 1`、暗黙バッチ（parallel/serial、debounce・モード変更による確定）、
開始姿勢の絶対角指令などはgoalの猶予を0（未設定）のままにしてcontroller既定を使います。
Humbleのjoint_trajectory_controllerは正のgoal猶予で`constraints.goal_time`を上書きし、
0なら既定を使うため、本パラメータを0にすると結果待ちの猶予はcontroller既定へ戻せます。
確定済み後続がある場合の計画時間での置換は、この猶予やfatal設定には依存しません。
後続なし・最後のcommit 0が結果を待つ場合の猶予として0.5秒の既定値を維持しています。
警告・失敗ログには理由と指令軸ごとの`target_deg`、`measured_deg`、`delta_deg`を1行で出します。
差は目標−実測（swingは最短角度差）です。未受信の実測は`unavailable`と表示します。

heartbeat受信前はリース監視を行いません。一度有効になると、全HTTP GET要求
（heartbeatやmotion_status、エラー応答になる要求も含む）がリースを更新します。
バッチ構築中・送信待ち・goal実行中に通信が途絶するとstop相当の処理を行い、
理由 `lease_expired` の `failed` にします。
送信期限切れと明示バッチの無通信期限切れは理由 `stale` です。
明示バッチの期限は最後のbegin／指令（速度設定を含む）からの無通信時間が上限を超えたときです。
1秒ごとに指令が届けば、beginからcommitまで5秒を超えても破棄しません。
reporterやheartbeatはバッチの指令期限を更新しません。
serialの2番目以降の指令は、直前の指令の終端から送信期限を数えます。

| パラメータ | 既定値 | 意味 |
|---|---|---|
| client_lease_sec | 2.0 | heartbeatで有効化後の通信途絶許容秒数 |
| max_dispatch_delay_sec | 1.0 | バッチ確定後の送信待ち上限秒数 |
| batch_open_timeout_sec | 5.0 | 明示バッチへ最後に指令が届いてからの無通信許容秒数（超過で破棄） |
| intermediate_tolerance_failure_fatal | false | commit 0のGOAL許容違反も致命的な失敗とする |
| intermediate_goal_time_tolerance_sec | 0.5 | 結果待ちするcommit 0の終端後猶予[秒]。置換時刻には影響しない。0はcontroller既定 |

`intermediate_goal_time_tolerance_sec`はROS Durationへ格納できる非負の有限数値
（最大2147483647秒）、その他の数値の追加パラメータは正の有限数値、
`intermediate_tolerance_failure_fatal`はboolで、同名のlaunch引数から設定できます。
単体launchでは`intermediate_goal_time_tolerance_sec:=0.5`で設定できます。
`tb20e_bringup`経由でも未指定時はこの既定値が適用され、既存の速度・位置許容差・停止速度の
YAML転送経路は変わりません。ただしbringupのYAML読み取り対象にはこの新パラメータが
含まれないため、`/**/scratch_hci_bridge.ros__parameters`に追加するだけでは転送されません。
相対指令は直前に成功または継続扱いとなった指令関節の目標角を基準とし、実測との差が上限を超える場合や
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
非指令軸も相対指令と同じく直前の指令目標で保持し、stop/reset/致命的失敗の後は実測角を使います。

### 保持目標への追従と軸マスク

`active_axis_mask_topic`（単体起動の既定は空、HTTP構成では
`/TB20e_0/http_active_axis_mask`）は、hardwareがレバー出力を許可する軸を指定します。
bit 0〜3は swing / boom / arm / bucket です。保持補正だけで無関係な軸を動かさないための
実機の安全機構ですが、指令軸だけを許可すると、短い区間で別の軸のgoalへ置き換えた際に
未到達の軸も停止してしまいます。

`catch_up_axes_in_mask`（bool、既定`true`）では、goal送信時に「そのgoalの指令軸 ∪
保持目標と最新実測の差が`trajectory_goal_tolerance_deg`を超える軸」を許可します。
旋回差は±180度内の最短角度差です。未到達軸は別軸のgoalへの置換後も追従を続け、
ノードのタイマー周期（`publish_hz`、既定50Hz）で許容内への到達を確認して解除します。
現在のgoalで指令中の軸はそのgoalの終了まで許可し、終了後は未到達軸だけを残します。
正常完了・非致命的な途中GOAL許容違反でも未到達軸の許可を維持します。
一度到達して解除した軸は、実測の変化だけでは再許可しません。次のgoal送信時に再判定します。

stop/reset、リース切れ、stale、致命的失敗、Actionタイムアウト、ノード終了では
追従情報を消去してマスクを0にします。最後のActionが終わって追従許可だけが残っている場合も
有効化済みのリース監視は継続します。idle時のタイマーはマスクを再送しないため、
単独`replay_motion`のマスクを上書きしません。

従来の指令軸だけの許可へ戻すには、単体launchで`catch_up_axes_in_mask:=false`、
またはNode起動時のROSパラメータに`catch_up_axes_in_mask: false`を指定します。
`tb20e_bringup`のYAML転送対象には含まれないため、controllers YAMLへの追記だけでは
切り替わりません。今回はbringup側の変更を行っていません。

### commit 0の区間接続

実行中のcommit 0に確定済み後続がある場合、送信時刻から`time_from_start`が経過した最初の
`_on_timer`で次のgoalを送ります。結果や`goal_time_tolerance`の猶予は待ちません。
判定の遅れは通常のタイマー実行では最大1周期（既定`publish_hz=50`なら20ms）です。
ROS通信・executor遅延・controller更新時間は別途加わります。後続が後から確定した場合は、
計画時間が既に過ぎていれば次のタイマーで送信します。後続なし、commit 1、最後の区間は結果を待ちます。
stop/reset、リース切れ、stale、cancel中は置換しません。新goalの応答待ちにstopした場合も、
受理後の新goalへcancelを要求し、終端までゲートを閉じます。

Humble JTCは新Actionの受理時に旧goalをpreemptし、旧goalへCANCELED（2.54.1ではcode=-1）を返します。
新goal受理より前に旧resultが届く場合は一旦保留し、受理が確定した置換の旧resultは無視します。
置換goalの拒否は致命的失敗とし、旧goalもcancelして終端を待ちます。
置換の受理応答が通信例外で不明になった場合は停止し、再送ゲートを閉じたままにします。

境界の位置は前の指令目標を使い、置換軌道には時刻0の始点と区間終点の位置・速度を指定します。
Humbleの`interpolation_method: splines`では位置＋速度の点間は3次補間になります。
区間速度は指令目標間の移動量÷区間時間です。既知の隣接区間が同方向なら両者の小さい速度を
境界に与え、反転・保持・クランプで移動しない軸は0にします。先読みした前goalの終点速度を
次goalの始点へ引き継ぎます。前goal送信時に後続が未知だった場合は、新区間速度で始めます。
さらに次の区間が未知、またはcommit 1の終点は速度0です。最初のcommit 0も送信時に後続が
既知なら速度付き2点を送り、境界へ向かう不要な減速を避けます。

終点速度を指定するにはJTCの`allow_nonzero_velocity_at_trajectory_end: true`が必要です。
インストール済みHumble 2.54.1は既定trueで、リポジトリのcontroller設定に上書きはありません。
3次補間の区間内ピーク速度は区間平均速度を超える場合があります。加速度・ジャークの制限や、
追従誤差が大きい場合の実機の連続性は、この接続処理だけでは保証しません。

仕様根拠: [Humbleのpreemption](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html#preemption-policy)、
[3次補間と軌道置換](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)。
