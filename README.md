# scratch_hci_bridge

ROS 2 Humble用のHCI HTTPブリッジです。PID関節 swing / boom / arm / bucket の動作を
関節別タイムラインへ追加し、4軸の `FollowJointTrajectory` goalへ変換します。
同じ関節はFIFO、異なる関節は同時に進みます。実行モードはScratchの変数が持ちます。

## ビルドと接続

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scratch_hci_bridge tb20e_motion_capture
source install/setup.bash
ros2 launch scratch_hci_bridge scratch_hci_bridge.launch.py
```

controllerと新鮮な4軸のJointStateが必要です。既定のActionは
`/tb20e_controller/follow_joint_trajectory`、JointStateは `/joint_states` です。
HCIの送信先は `http://localhost`、port `8899`。別PCではROS側のIPを指定します。
接続先設定はScratch内部の操作です。commandブロックが通信完了を待つHCI拡張を使用してください。
HTTPは到着順に処理し、stop/resetは待機列を追い越して適用します。

## HTTP指令

形式は `GET /?type=command&cmd=<指令>&val=<有限数値>`。
成功は200 / `OK`、不正な指令・値・待機列超過は400です。

| 指令 | valと動作 |
|---|---|
| swing_left / swing_right | 移動量[deg]。正方向 / 負方向 |
| boom_up / boom_down | 移動量[deg]。負方向 / 正方向 |
| arm_extend / arm_retract | 移動量[deg]。負方向 / 正方向 |
| bucket_dump / bucket_scoop | 移動量[deg]。負方向 / 正方向 |
| swing_angle / boom_angle / arm_angle / bucket_angle | 絶対目標[deg]への区間を追加 |
| swing_speed / boom_speed / arm_speed / bucket_speed | 後から受け付ける区間の速度[deg/s]。0で未設定へ戻す |
| barrier | val未使用。全関節の受付済み区間の最大終了時刻を保存 |
| blade_angle / body_angle / thumb_angle | 個別topicへ直接送る目標[deg] |
| left_crawler_speed / right_crawler_speed | 左右走行速度 |
| stop | val未使用。走行ゼロ、計画破棄、goalをcancel |
| reset | val未使用。停止処理と失敗ラッチ解除 |

相対指令のvalは符号を無視します。旋回の相対量180度以上は拒否します。
旋回170度から左30度は200度への連続角となり、逆方向へ回りません。
絶対旋回は±180度へクランプし、直前目標からの最短経路を使います。
可動域はboom=-83〜48、arm=32〜155、bucket=-31〜159、blade=-30〜15、thumb=0〜140度。
bodyはクランプしません。

区間の起点は同関節の直前の区間の目標です。区間がなければ直前の指令目標を使い、
未指令の関節・stop/reset・致命的失敗の後は実測を使います。
所要時間は受付時の速度があれば「クランプ後の移動量÷速度」、未設定なら
`trajectory_duration_sec`。最短時間は0.001秒です。受付後の速度変更は既存区間へ影響しません。
feedback未受信中の指令は速度と順序を保持し、feedbackが揃った時点で角度と時刻を解決します。
ROS Durationで表現できない時間は拒否します。

## barrierと先行送信

区間の開始は `max(受付時刻, 保存した区切り時刻, 同関節の直前終了)` です。
barrierは計画時刻を保存するだけで、停止結果を待ちません。
短い関節は自分の目標を保持し、区切りまで待ちます。

```text
boom_speed 10
arm_speed 10
boom_down 20       # boom: 0〜2秒
arm_extend 10      # arm: 0〜1秒
barrier           # 区切り=2秒
arm_extend 5      # arm: 2〜2.5秒（2秒前に送ってよい）
```

受付時刻は説明用にすべて0秒としています。実際にはHTTPの到着時刻を使います。
同関節の終了と次の開始が同じなら、同方向の境界速度は小さい方、反転・保持は0。
barrierを挟んでも同じ時刻に接続すれば止まりません。
後続追加と区間開始でgoalを置き換えます。既存区間の角度と終了時刻、過去のbarrierは変えず、
置換始点は計画上の現在位置です。全関節の境界の和集合を軌道点に使います。
送信はpublish_hzのタイマーとgoal受理応答のゲートを通るため、同周期内の追加は一つにまとまります。

Scratchの「動作モード：シリアル／パラレル」は変数変更とbarrier送信です。
動作ブロックは対応するjoint_queueが0になるまで安全確認付きで待ち、serialなら追加のbarrierを送り、
動作を送って次へ進みます。「止まるまで待つ」だけが実際の停止結果を待ちます。

## reporterと安全停止

形式は `GET /?type=reporter&cmd=<名前>&val=<値>`。

| 名前 | val | 本文 |
|---|---|---|
| joint_queue | 0 / 1 / 2 / 3 | swing / boom / arm / bucketの未開始区間数。整数文字列 |
| motion_status | 0 | idle / busy / failed / stopped |
| motion_error | 0 | 最初の失敗理由。resetまで保持 |
| heartbeat | 0 | ok。リース監視を有効化・更新 |

未開始区間は関節ごと1件まで。実行中の区間は数えません。
feedback待ちの指令は未開始として数えます。joint_queue=0は全停止の意味ではありません。
`busy`は計画が残っている間と最後のgoalの結果待ちです。最後のgoalは位置許容と停止速度許容を
満たす成功結果でidleになります。途中で意図的に置換した旧goalの結果は失敗にしません。
failedは致命的失敗、stoppedはstop後（cancel終端待ちはbusy）、reset後はidleです。

feedback途絶、送信待ち期限、リース切れ、Action拒否・許容違反・結果timeoutで計画を消去し、
出力マスクを0、走行を0にしてcancelします。cancel応答だけでは完了にせず、goal終端までゲートを閉じます。
heartbeatを一度受けるとリース監視が有効になり、その後のHTTP要求でも更新します。
出力マスクは動作中の軸と未到達の追従軸を許可し、未来の待機区間だけで軸を許可しません。

## 主なROSパラメータ

| 名前 | 既定 | 用途 |
|---|---:|---|
| http_host / http_port | 0.0.0.0 / 8899 | HTTP接続 |
| excavator / publish_hz | tb20e / 50 | 走行namespaceと処理周期[Hz] |
| trajectory_action | /tb20e_controller/follow_joint_trajectory | Action |
| joint_state_topic | /joint_states | 4軸の実測[rad] |
| trajectory_duration_sec | 3.0 | 速度未設定時の区間時間[s] |
| 各関節_default_speed_deg_s | 0.0 | PID4軸の初期速度設定。0で未設定 |
| joint_state_timeout_sec | 0.5 | feedback鮮度[s]。0で監視無効 |
| trajectory_goal_tolerance_deg | 0.75 | 最後の位置許容 |
| trajectory_stopped_velocity_deg_s | 0.5 | 最後の停止速度許容 |
| max_dispatch_delay_sec | 1.0 | 最初のgoalの送信待ち上限[s] |
| client_lease_sec | 2.0 | heartbeat有効化後の無通信上限[s] |
| trajectory_goal_response_timeout_sec | 5.0 | 受理応答待ち上限[s] |
| trajectory_result_timeout_margin_sec | 35.0 | 計画終了後の結果待ち猶予[s] |
| trajectory_shutdown_cancel_timeout_sec | 2.0 | 終了時のcancel待ち上限[s] |
| active_axis_mask_topic | 空 | UInt8の許可軸マスク。空で無効 |
| catch_up_axes_in_mask | true | 未到達軸の追従を継続 |
| direct_joint_command_prefix | /TB20e | blade/body/thumbの個別topic prefix |
| drive_linear_scale / drive_angular_scale | 1.0 / 1.0 | 走行Twist倍率 |
| unity_position_output_enabled | false | 受理目標のUnity位置topicへの追加出力 |
| unity_position_command_prefix | /TB20e | Unity位置topic prefix |
| swing_unity_position_sign | -1.0 | Unity位置出力の旋回符号 |
| boom_unity_position_sign / arm_unity_position_sign / bucket_unity_position_sign | 1.0 | Unity位置出力の符号 |

走行は `linear.x=(right+left)/2*drive_linear_scale`、
`angular.z=(right-left)/2*drive_angular_scale` です。PIDタイムラインとは独立です。
Unityのレバー経路で追従を確認する場合、位置出力の追加機能は無効にします。

## 単体検証

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/scratch_hci_bridge /usr/bin/python3 -m pytest src/scratch_hci_bridge/test -q
```

テストはダミーActionとローカルHTTPだけを使い、Unity・実機・常駐ノードを起動しません。

## LiDAR距離・接地とガード付き下降

`tb20e_ground_sensing` と同時に使用する。`ground_topic_prefix`（既定 `/TB20e_0/ground`）の
`clearance`（std_msgs/Float64、m、NaN=unknown）、`contact`（std_msgs/Int8、-1=unknown、0/1）
を購読する。`ground_timeout_sec`（既定0.5秒）を過ぎた値はunknownとして扱う。
launchでも両パラメータを指定できる。

| reporter（val=0） | 本文 |
|---|---|
| ground_clearance | m、小数3桁。非有限値・受信途絶は `unknown` |
| ground_contact | `1` / `0` / `unknown` |
| guard_status | `running` / `contact` / `reached` / `failed` |

`boom_down_until_contact`（val=最大下げ角度[deg]、符号無視）は、boom_downと同じ速度・可動域・
FIFO・barrierで区間を予約する。新鮮な接地情報と4軸feedbackが揃わない開始は400で拒否し、
既存の安全停止経路で `ground_guard_rejected` をラッチする。reset後に再試行する。
開始前の未知領域へブームを下げると停止根拠を持てないため、unknownのまま進めない。

ガード実行中は追加のboom指令を拒否する。直接HTTPで拒否するとguardは継続するが、
Scratchのガード二重起動など、ガード開始要求の拒否は安全停止する。
区間がbarrierで未来に予約された場合は、開始時刻から接地による保持を適用する。
接地した時点の実測boom位置・速度0へgoalを置換し、boomの残りの区間を捨てる。
他軸の計画は維持する。センサーcallbackで保持を予約し、publish_hzとAction受理ゲートで送信する。
軸マスク有効時は接地直後から保持goal受理までboomを禁止し、旧goalへの追従を抑える。
保持goalの受理時に実測差から保持出力を再評価する。マスクなしではgoal置換の受理まで旧goalが動き得る。
実際の停止距離・追従遅れをゼロにはできないので、Unityで動作速度ごとに確認する。

`contact` は保持計画への変更時、`reached` は最新Actionの正常終端時に返す。
`contact` は物理的な停止完了を意味しない。必要なら既存の「止まるまで待つ」で終端も待つ。
複数軸をHTTPから後続追加した場合、reachedはその最新goalの終端まで待つ。
unknown通知・接地情報途絶・feedback異常で `failed` とし、計画破棄・走行ゼロ・cancelを行う。
stopは実行中ガードをfailed、resetはガードをreached（実行なし）へ戻す。
リース、Action拒否・失敗・timeout、cancel終端ゲートは既存の停止経路を共有する。
