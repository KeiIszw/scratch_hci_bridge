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
launchでもこれらのパラメータを指定できる。

| reporter（val=0） | 本文 |
|---|---|
| ground_clearance | m、小数3桁。非有限値・受信途絶は `unknown` |
| ground_contact | `1` / `0` / `unknown` |
| guard_status | `running` / `contact` / `stalled` / `reached` / `blocked` / `failed` |

`boom_down_until_contact`（val=最大下げ角度[deg]、符号無視）は、boom_downと同じ速度・可動域・
FIFO・barrierで区間を予約する。新鮮な接地情報と4軸feedbackが揃わない開始は400で拒否し、
既存の安全停止経路で `ground_guard_rejected` をラッチする。reset後に再試行する。
開始前の未知領域へブームを下げると停止根拠を持てないため、unknownのまま進めない。

ガード実行中は追加のboom指令を拒否する。直接HTTPで拒否するとguardは継続するが、
Scratchのガード二重起動など、ガード開始要求の拒否は安全停止する。
区間がbarrierで未来に予約された場合は、開始時刻から接地による保持を適用する。
実行区間で新鮮な`ground_clearance`が`ground_slow_distance_m=0.2`m未満になったら、
残りのboom下降を`ground_slow_speed_deg_s=1.0`°/s以下へ一度だけ減速する。両値はlaunchでも設定可能。
計画現在位置と実測のうち下降側に進んだ方を新しい起点にし、上方向へ戻さない。
実測が終点を越えていれば実測位置で止める。距離が増えても再加速しない。既に十分遅い区間は変更しない。
実測へ起点を変更した場合は始端速度0でつなぎ、上向きの位置・速度を生成しない。
JTCの3次補間で最大速度が平均の1.5倍になるため、残所要時間は1.5×残角度÷上限速度とする。
減速で延びた時間（負なら0）だけ、旧終点以降に始まる未来区間と保存barrierを後ろへ送る。
開始済みの他軸は継続し、barrier後の区間が下降完了前に先行することを防ぐ。
減速後の追加指令にも更新済みのbarrierを使う。新鮮な距離が未受信なら減速は行わず、
接地のunknown/途絶による既存停止を引き続き使う。

終了理由は次の状態で区別する。`guard_status`は保持の予約時点で変わり、停止完了は
`motion_status=idle`で確認する。正常な打切りではresetなしで次の指令を受け付ける。

| 状態 | 意味 | motion failedラッチ |
|---|---|---|
| running | barrier待ち、または下降中 | なし |
| contact | 推定接地（ground_sensingで連続観測・急落除去済み） | なし |
| stalled | 新鮮な距離の直近5観測中央値が0.2m未満でboomが停滞。接地扱いで保持 | なし |
| reached | 指定角度へ到達し、推定接地なし。実測位置で保持 | なし |
| blocked | 距離中央値が0.2m以上で停滞。障害物の可能性として保持 | なし |
| failed | センサーunknown/途絶、角度途絶、Action拒否などの異常 | あり、reset必須 |

停滞は`ground_stall_sec=0.5`秒以上の実測boom角の全振幅が
`ground_stall_epsilon_deg=0.1`°以下の場合。繰り返し読むtimerは観測に数えない。
JTC相当の3次補間位置が実測より下降側へ0.1°を超えて進んでいる観測だけを数える。
計画が実測の後ろにある間は窓を毎回リセットし、追いつき待ちの時間を停滞へ加算しない。
減速置換時にも旧窓を捨てる。窓内の予定変位>0.2°（0.5秒で平均>0.4°/s）も必要とし、
3次補間の始動待ちを除外する。計画終了後は未到達量>0.1°の停止を検出する。
静止幅0.1°/0.5秒は正味の平均速度0.2°/s以下に相当するが、往復も検出するため全振幅で判定する。
2026-11-12の3試行の保持後実測振幅と受信周期は`codex_runs/20261112_guard_fix2/report.md`を参照する。
距離は刃先だけでなく有効バケットコライダーの形状最小であり、背面が先に当たる姿勢も分類へ反映する。
近い/遠いの分類には新鮮な有限距離を必須とし、不明ならfailedとする。
両停滞パラメータはlaunchでも設定可能。地面近くの物理障害物もstalledになるため、
真の接地を保証する状態ではない。必要に応じ「接地動作状態」で分岐する。

接地・停滞・到達でboomの残りの区間を捨て、実測角・速度0の保持goalへ置換する。
他軸の計画は維持する。検知後にさらに下がった角度へ保持goalを追従させ、0.5秒の
静止確認後に固定する（下降はboom角の正方向）。検知位置へ上向きに戻す目標は作らない。
静止確認中の追加boomは拒否するので、HTTPはidleを待ってから送る。
Scratchの接地ブロックは終了理由を変数「接地動作状態」に保存し、保持Actionと静止確認の
完了も待ってから次へ進む。正常4状態では「特殊：停止」を呼ばない。

軸マスク有効時は検知直後・古い保持goal受理待ち・実測が旧保持角を越えた時点でboomを禁止する。
JointState callbackでも即時に再評価し、古い角への上向き追従を再許可しない。
Unity位置出力にも遅着した古い保持角を流さない。マスクなしのJTCへ既に送ったgoalは
後から書き換えられず、通信/受理中の追従を完全には抑止できない。実際の停止距離はUnityで確認する。

JTCの許容超過は、新鮮な実測停滞から保持へ移行でき、他の指令軸が0.75°以内に到達済みの場合、
またはその保持より古いgoalの結果の場合だけ回復する。それ以外の失敗を隠さない。
最終保持goalそのものの失敗、unknown・feedback異常、リース切れ、Action拒否・timeoutは
計画破棄・走行ゼロ・cancelを行いfailedをラッチする。停止根拠や制御の成立が不明なため、
Scratchも停止してreset後の再確認を求める。stopは実行中ガードをfailed、resetはreachedへ戻す。
stop/reset/failedで古い終点・保持目標を破棄し、cancel終端までは既存の送信ゲートを維持する。
