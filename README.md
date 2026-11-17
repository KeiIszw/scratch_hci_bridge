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
| barrier | 0・省略: 計画終了の区切り。1: 計画終了と実測到着を待つゲート。その他は拒否 |
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
未指令の関節・stop/resetの後は実測を使います。抵抗停止後は固定した停止位置を使います。
stopは次の動作指令で自動解除し、異常failedはresetが必要です。
所要時間は受付時の速度があれば「クランプ後の移動量÷速度」、未設定なら
`trajectory_duration_sec`。最短時間は0.001秒です。受付後の速度変更は既存区間へ影響しません。
feedback未受信中の指令は速度と順序を保持し、feedbackが揃った時点で角度と時刻を解決します。
ROS Durationで表現できない時間は拒否します。

## barrierと先行送信

区間の開始は `max(受付時刻, 保存した区切り時刻, 同関節の直前終了)` です。
`barrier val=0`（val省略も同じ）は計画時刻を保存するだけで、停止結果を待ちません。
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

`barrier val=1`は、それまでの全区間が計画終了し、指令済み各関節の実測が最終目標から
軸別の位置許容以内、かつ実測角速度が`serial_arrival_speed_deg_s=2.0`°/s未満になるまで、
後から受け付ける区間を保留します。既定の位置許容はswing 1.5°、boom 2.5°、arm/bucket 5.5°。
boomは記録の静止残差約1.95°を受け入れる余裕を取っています。
指令済み全軸を判定し、未指令軸の到着目標は作りません。
角速度はフィードバック位置の隣接差分の絶対値を0.1秒窓（境界直前の1サンプルを含む）で積算し、経過秒で割ります。
旋回の±360°のwrapを除き、反転が相殺されない形で平滑化します。
初回や0.2秒超の途絶・時計逆行の後は、0.1秒以上の連続観測が揃うまで通しません。
判定はタイマー周期（既定20ms）で行い、解除時の現在時刻から後続を開始します。
2°/s未満なので厳密な完全静止を保証する条件ではありません。一度通したゲートは閉じ直しません。
通常到着で通過した軸は、次の明示指令まで実測位置を保持出力に使い、出力マスクによる再許可も抑止します。
通常到着した軸の次の相対指令の**終点**は従来の理想目標から計算し、**始点と所要時間**を受け入れた実測位置に合わせます。
抵抗で停止した軸は未達目標を捨て、固定した停止位置を能動保持し、次の同軸指令の始点・相対終点の基準にも使います。
これにより他軸追加時の保持補正を抑え、往復指令の残留誤差を累積させません。
保留中は速度と受付順を保持し、Actionの軌道・出力許可の対象に含めず、joint_queueには数えます。
複数ゲート、従来barrier、feedback待ちの受付も順序を保ちます。後続のval=0で未解除ゲートを解除しません。

到着待ちの期限は「計画終了とゲート適用時刻の遅い方」から`serial_arrival_timeout_sec=10.0`秒です。
計画時間そのものはこの到着待ち期限へ含めません。抵抗停止を確定した軸は実測保持へ置換し、
到着済みとして扱います。下記の抵抗条件を満たさない許容外静止は自動続行せず、10秒で異常終了します。
別に`motion_timeout_sec=240.0`秒の全体期限があり、最初の指令受付からidleまでを計測します。
計画・接地減速・ゲート待ち・抵抗保持・結果待ちを含み、追加指令や保持置換では延長しません。
超過時は後続へ進めず、`motion_status=failed` / `motion_error=serial_arrival_timeout`をラッチし、
保留・計画を破棄、走行ゼロ・出力マスク0・goal cancelへ進みます。resetでラッチを解除してください。
feedback途絶、Action失敗、stop/resetも保留とゲートを破棄し、送信済みgoalの終端待ちは維持します。
前半だけのAction成功では保留を捨てず、到着待ち中に同じ保持goalを繰り返し送りません。
接地ガードの待機中は下降を開始せず、実行中の減速で延びた計画終了も待ちます。
contact/stalled/reached/blockedで正常打切りした場合は、元の下降目標を捨てた後の実測保持目標で判定します。

Scratchの「動作モード：シリアル」は変数変更とbarrier val=1送信、パラレルはval=0送信です。
相対動作・角度ブロックは対応するjoint_queueが0になるまで安全確認付きで待ち、serialなら
barrier val=1を送り、動作を送って次へ進みます。接地動作もserialではval=1、parallelではval=0で区切ります。
「特殊：姿勢初期化」は呼出し元の実行モードを退避し、並列で初期姿勢へ移動して停止完了を待ち、
元のモードへ戻します。「止まるまで待つ」は最後のAction成功とブリッジの軸別位置・実測速度条件の両方を待ちます。

## reporterと安全停止

形式は `GET /?type=reporter&cmd=<名前>&val=<値>`。

| 名前 | val | 本文 |
|---|---|---|
| joint_queue | 0 / 1 / 2 / 3 | swing / boom / arm / bucketの未開始区間数。整数文字列 |
| motion_status | 0 | idle / busy / stopped / failed。stopは次の動作指令で自動解除 |
| motion_error | 0 | 最初の失敗理由はresetまで保持。通常stopのstoppedは次の動作指令で消去 |
| heartbeat | 0 | ok。リース監視を有効化・更新 |
| joint_angle | 0 / 1 / 2 / 3 | swing / boom / arm / bucketの実測[deg]、小数3桁。欠測・非有限・期限切れはunknown |
| stall_joints | 0 | 抵抗停止した関節名をswing,boom,arm,bucket順のカンマ区切り。未検出はnone |
| stall_position | 0 / 1 / 2 / 3 | その軸の最新抵抗停止位置[deg]、小数3桁。未検出はunknown |
| motion_outcome | 0 | running / arrived / resistance / stopped / abnormal |

未開始区間は関節ごと1件まで。実行中の区間は数えません。
feedback待ち・到着ゲート待ちの指令は未開始として数えます。joint_queue=0は全停止の意味ではありません。
`busy`は計画・保持・ブリッジの到着確認または最後のAction結果が残る間です。
Action成功だけでidleにはせず、計画終了、新鮮な4軸feedback、指令軸の軸別位置許容と
実測速度<2°/sを確認します。Action成功後に未到着なら再送を繰り返さず観測を待ちます。
途中で意図的に置換した旧goalの結果は失敗にしません。異常は`failed`をラッチします。
**stopは通常停止としてstoppedを公開し、次の動作指令で自動解除します。resetは不要です。**
受付済みの新指令もcancel終端までは新goalを送りません。既存failedをstopで解除することはできません。HTTPの通常commandはfailed中に400で拒否し、走行・直結関節の状態も更新しません。
reporterとreset/stopは受け付けます。旧共通`trajectory_goal_tolerance_deg`は廃止しました。

### 掘削抵抗による正常打切り

新しいJointState観測ごとに、送信済み軌道の3次Hermite補間位置と実測を比較します。
その関節の区間の指令方向へ計画が実測より`<axis>_load_stall_lead_deg`以上先行している間の、
直近`<axis>_load_stall_window_sec=1.5`秒の実測角の最大−最小が`stall_motion_deg=0.5`°未満なら抵抗停止です。
微振動の瞬間速度で窓を切りません。先行差不足・逆向き誤差・0.2秒超のサンプル間隔・
同時刻・時刻逆行・非有限値では窓をリセットし、timerの再読込は観測に数えません。
窓の左端をまたぐ1観測を含めて保守的に評価するため、周期1回分ほど遅れる場合があります。

先行差の初期値はswing/boom=2.5°、arm/bucket=8.5°です。controllersのlever_start数値+0.5を
保守的な初期目安としています。**lever_start/stopの実際の単位はレバー%で、角度への単位換算ではありません。**
bringupはHTTP節の明示角度値を優先します。PID/FF/負荷に応じた実機較正が必要です。
角度幅0.5°はdig3.csvの接触後の微振動・緩い沈み込み（boom約0.43°、arm後半約0.24°）を覆う初期値です。
先行差が大きいまま1.5秒で0.5°未満しか進まない意図的な低速動作も、この入力だけでは抵抗と区別できません。
armの接触直後の約1°の進行は止まった扱いにせず、幅が狭まるまで検出を待ちます。
旧`load_stall_sec` / `load_stall_speed_deg_s`は廃止し、各軸の窓秒数 / 共通の角度幅へ移行しました。

boom/arm/bucketの抵抗停止では、その軸の実行中・未来の区間を捨て、**検出時の実測位置そのもの・速度0**の
保持goalへ置換します。保持位置はドリフト・到着ゲート・idle化・他軸の指令で更新しません。
保持goalの受理後は出力マスクを維持し、他軸のgoalでも全軌道点に同じ保持角と速度0を含め、Action終点許容も有効にします。
旧未達goalから保持goalへの受理待ち中だけは、旧目標への押し込みを避けるためその軸を一時抑止します。
idle後も能動保持を続け、HTTPリース・実測鮮度の監視対象にします。
他軸の区間・既存barrier時刻は保ち、ゲート後の保留指令は後続の明示指令として受け付けます。
次の同軸明示指令は固定した停止位置を起点に通常動作へ戻り、その軸の能動保持を解除します。
通常到着軸の出力抑止は従来どおりです。stop/reset/異常停止では能動保持と保留計画を破棄します。
抵抗停止の履歴reporterはstopでは残し、resetで消去します。
**swingの抵抗停止は障害物衝突の可能性があるため、全軸停止・failed / swing_load_stallです。**
接地ガード中・保持静止確認中のboomは従来のground guard判定を優先し、この判定から除きます。

`motion_outcome`は関節指令受付でrunning、到着完了でarrived、正常抵抗打切りでresistance、通常stopでstopped、異常でabnormal。
連続するbusy区間で一度resistanceになればidleまで維持し、保持・他軸の完了は`motion_status=idle`で確認します。
後続の新しいbusy区間はrunningへ戻ります。`stall_joints`と`stall_position`は各軸の最新検出をresetまで保存し、
現在も止まっているという意味には使いません。swingの異常停止位置も記録します。
全体期限超過は`motion_timeout`、非有限角度は直ちに`nonfinite_feedback`です。
抵抗停止後の保持goal自体の拒否・許容違反・通信異常はfailedであり、抵抗として隠しません。

feedback途絶・非有限、送信待ち期限、全体期限、リース切れ、Action拒否・許容違反・結果timeoutで計画を消去し、
出力マスクを0、走行を0にしてcancelします。cancel応答だけでは完了にせず、goal終端までゲートを閉じます。
heartbeatを一度受けるとリース監視が有効になり、その後のHTTP要求でも更新します。
出力マスクは動作中の軸と未到達の追従軸を許可し、未来の待機区間だけで軸を許可しません。

位置許容・速度上限・期限はいずれも正の有限値です。旧`serial_arrival_tolerance_deg`は廃止しました。
bringupではcontrollers YAMLのHTTP節から設定できます（bringup README参照）。

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
| swing_serial_arrival_tolerance_deg | 1.5 | val=1の旋回位置許容[deg] |
| boom_serial_arrival_tolerance_deg | 2.5 | val=1のboom位置許容[deg] |
| arm_serial_arrival_tolerance_deg / bucket_serial_arrival_tolerance_deg | 5.5 | val=1のarm/bucket位置許容[deg] |
| serial_arrival_speed_deg_s | 2.0 | val=1の平滑実測角速度上限[deg/s]。未満で通過 |
| serial_arrival_timeout_sec | 10.0 | 計画終了後の到着待ち上限[s]。正の有限値 |
| 各軸_load_stall_lead_deg | 2.5 / 2.5 / 8.5 / 8.5 | 抵抗判定を始める指令方向の先行差[deg] |
| 各軸_load_stall_window_sec | 1.5 | 抵抗を判定する実測角の窓[秒] |
| stall_motion_deg | 0.5 | 窓内の最大−最小の上限[deg]（未満で停止） |
| motion_timeout_sec | 240.0 | 最初の受付からidleまでの全体上限[s] |
| trajectory_stopped_velocity_deg_s | 2.0 | JTCの停止速度許容[deg/s]。bridgeはserial_arrival_speed_deg_sで判定 |
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
Scratchも停止してreset後の再確認を求める。通常stopはmotionをstopped、実行中ガードはfailedにする。
次の動作指令でmotionのstoppedを自動解除する。ガードも新しい明示指令で再開でき、reset必須ではない。resetはガードをreachedへ戻す。
stop/reset/failedで古い終点・保持目標を破棄し、cancel終端までは既存の送信ゲートを維持する。

JTCへ送る終点位置許容は到着ゲートと同じ軸別値です。途中位置許容は120°、goal_timeは30秒を
Actionに明示し、0や-1による無効化はしません。controllers_0.yamlも同じ値です。
120°は通常の60°/s×1.5秒=90°に置換通信の余裕を取った緩い最終限界で、停止の主判定には使いません。
独立した実機hardwareのレバー制限・可動域・速度保護は維持します。
