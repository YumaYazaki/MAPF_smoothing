* `SpatialPathDescriptor` は **幾何の正本**
* `IntentProfile` は **上位意図の正本**
* B社はこの2つを入力にして、装置制約を踏まえて最終 retiming を行う
* hard / soft の区別は **IntentProfile 側**で持つ

---

# 1. 全体像

| オブジェクト                  | 役割      | 主な中身                      | 正本性   |
| ----------------------- | ------- | ------------------------- | ----- |
| `SpatialPathDescriptor` | どこを通るか  | 経路幾何、区間、曲率、接線、evaluator情報 | 幾何の正本 |
| `IntentProfile`         | どう通したいか | 速度レンジ、時間窓、優先度、停止可否、協調制約   | 意図の正本 |

---

# 2. SpatialPathDescriptor 定義

## 2.1 Path-level ヘッダ

| 項目名                      | 型          | 必須 | 説明          | 備考                                     |
| ------------------------ | ---------- | -: | ----------- | -------------------------------------- |
| `path_id`                | string     |  ○ | 経路ID        | 一意識別子                                  |
| `robot_id`               | string     |  ○ | 対象ロボットID    |                                        |
| `task_id`                | string     |  ○ | 搬送タスクID     |                                        |
| `version`                | string     |  ○ | データ仕様バージョン  | 互換性管理用                                 |
| `frame_id`               | string     |  ○ | 座標系ID       | 例: `world`                             |
| `s_start`                | float      |  ○ | 全体経路の開始アーク長 | 通常 0.0                                 |
| `s_end`                  | float      |  ○ | 全体経路の終了アーク長 | 通常 = `total_length`                    |
| `total_length`           | float      |  ○ | 全体経路長       | 単位は要合意                                 |
| `path_generation_method` | enum       |  △ | 経路生成法       | 例: `mapf_plus_quintic_local_smoothing` |
| `geometry_confidence`    | enum/float |  △ | 幾何信頼度       | 任意                                     |
| `validity_horizon`       | float      |  △ | 有効期間        | 動的環境向け                                 |

---

## 2.2 geometry_segments

各 segment は、全体 `s` 軸上の一部分を表す。

### 共通項目

| 項目名               | 型        | 必須 | 説明        | 備考                       |
| ----------------- | -------- | -: | --------- | ------------------------ |
| `segment_id`      | int      |  ○ | 区間ID      | 経路内一意                    |
| `geometry_type`   | enum     |  ○ | 幾何型       | `line`, `quintic_corner` |
| `s_start`         | float    |  ○ | 区間開始アーク長  | 全体 `s` 基準                |
| `s_end`           | float    |  ○ | 区間終了アーク長  | 全体 `s` 基準                |
| `length`          | float    |  ○ | 区間長       | `s_end - s_start`        |
| `samples_ref_id`  | string   |  △ | 対応サンプル列ID | 補助用                      |
| `prev_segment_id` | int/null |  △ | 前区間ID     |                          |
| `next_segment_id` | int/null |  △ | 次区間ID     |                          |

---

### geometry_type = `line`

| 項目名         | 型        | 必須 | 説明       | 備考     |
| ----------- | -------- | -: | -------- | ------ |
| `p_start`   | float[2] |  ○ | 始点座標     |        |
| `p_end`     | float[2] |  ○ | 終点座標     |        |
| `tangent`   | float[2] |  ○ | 単位接線ベクトル | 定数     |
| `curvature` | float    |  ○ | 曲率       | 通常 0.0 |

---

### geometry_type = `quintic_corner`

| 項目名                | 型        | 必須 | 説明            | 備考                           |
| ------------------ | -------- | -: | ------------- | ---------------------------- |
| `corner_point`     | float[2] |  ○ | 元の折れ点座標       |                              |
| `q_minus`          | float[2] |  ○ | smoothing 開始点 |                              |
| `q_plus`           | float[2] |  ○ | smoothing 終了点 |                              |
| `e_in`             | float[2] |  ○ | 進入単位方向        |                              |
| `e_out`            | float[2] |  ○ | 退出単位方向        |                              |
| `d`                | float    |  ○ | 接続長           |                              |
| `lam`              | float    |  ○ | 接線スケール        |                              |
| `x_coeffs`         | float[6] |  ○ | 局所 x(u) 5次係数  | `a0..a5`                     |
| `y_coeffs`         | float[6] |  ○ | 局所 y(u) 5次係数  | `b0..b5`                     |
| `length_L`         | float    |  ○ | corner 区間長    | `u=0..1` の弧長                 |
| `table_u`          | float[]  |  ○ | 単調 u テーブル     | `u_0=0, u_N=1`               |
| `table_s`          | float[]  |  ○ | 単調 s(u) テーブル  | `s_0=0, s_N=length_L`        |
| `quadrature_order` | int      |  △ | A社側で使った数値積分設定 | 共有すると再現性向上                   |
| `evaluator_type`   | enum     |  △ | `u(s)` 解法種別   | 例: `hybrid_newton_bisection` |

---

## 2.3 path_samples（補助情報）

これは正本ではなく、初期実装・デバッグ用として使用する。

| 項目名             | 型     | 必須 | 説明      | 備考 |
| --------------- | ----- | -: | ------- | -- |
| `sample_id`     | int   |  ○ | サンプルID  |    |
| `s`             | float |  ○ | 全体アーク長  |    |
| `x`             | float |  ○ | 座標 x    |    |
| `y`             | float |  ○ | 座標 y    |    |
| `dx_ds`         | float |  △ | 接線 x 成分 |    |
| `dy_ds`         | float |  △ | 接線 y 成分 |    |
| `kappa`         | float |  △ | 曲率      |    |
| `segment_id`    | int   |  ○ | 所属区間ID  |    |
| `geometry_type` | enum  |  △ | 区間型     |    |

---

# 3. IntentProfile 定義
---

## 3.1 Path-level intent

| 項目名                 | 型          | 必須 | 説明         | 備考                                                     |
| ------------------- | ---------- | -: | ---------- | ------------------------------------------------------ |
| `path_id`           | string     |  ○ | 対応する経路ID   | `SpatialPathDescriptor.path_id` と一致                    |
| `global_priority`   | enum/int   |  △ | 経路全体の優先度   | 例: `low/normal/high`                                   |
| `energy_mode`       | enum       |  △ | 運用モード      | `throughput`, `balanced`, `eco`                        |
| `stop_policy`       | enum       |  △ | 全体停止方針     | `allowed`, `discouraged`, `forbidden_except_emergency` |
| `replanning_policy` | enum       |  △ | 再計画方針      |                                                        |
| `intent_confidence` | enum/float |  △ | 意図の強さ/確信度  |                                                        |
| `validity_horizon`  | float      |  △ | この意図が有効な期間 |                                                        |

---

## 3.2 Segment-level intent

幾何区間に対して、どう通したいかを指定する。

| 項目名                      | 型           | 必須 | 説明      | 備考                                                           |
| ------------------------ | ----------- | -: | ------- | ------------------------------------------------------------ |
| `segment_id`             | int         |  ○ | 対応区間ID  | `geometry_segments.segment_id` と対応                           |
| `segment_type`           | enum        |  ○ | 区間意味    | 例: `straight_like`, `turning`, `coordination`, `positioning` |
| `constraint_class`       | enum        |  ○ | 制約クラス   | `H`, `S1`, `S2`                                              |
| `v_min`                  | float/null  |  △ | 下限速度    | A社意図上の下限                                                     |
| `v_nominal`              | float/null  |  △ | 代表速度    | 希望値                                                          |
| `v_max`                  | float/null  |  △ | 上限速度    | A社要求上の上限                                                     |
| `enter_window_start`     | float/null  |  △ | 進入時刻窓開始 | 全体時刻基準                                                       |
| `enter_window_end`       | float/null  |  △ | 進入時刻窓終了 |                                                              |
| `exit_window_start`      | float/null  |  △ | 退出時刻窓開始 |                                                              |
| `exit_window_end`        | float/null  |  △ | 退出時刻窓終了 |                                                              |
| `priority_local`         | enum/int    |  △ | 局所優先度   |                                                              |
| `stop_allowed`           | bool        |  △ | 停止許容    | `false` なら停止回避意図が強い                                          |
| `dwell_allowed`          | bool        |  △ | 滞留許容    |                                                              |
| `overtake_forbidden`     | bool        |  △ | 追越禁止    | 必要時                                                          |
| `coordination_region_id` | string/null |  △ | 協調領域ID  | 共有区間識別                                                       |
| `notes`                  | string      |  △ | 人間向け補足  |                                                              |

---

## 3.3 Event-level intent
segment 全体ではなく、「どの点をいつどう通ってほしいか」を表す。

| 項目名                          | 型           | 必須 | 説明         | 備考                                                                             |
| ---------------------------- | ----------- | -: | ---------- | ------------------------------------------------------------------------------ |
| `event_id`                   | string      |  ○ | イベントID     | 一意識別子                                                                          |
| `path_id`                    | string      |  ○ | 対応経路ID     |                                                                                |
| `event_type`                 | enum        |  ○ | イベント種別     | `enter_checkpoint`, `exit_checkpoint`, `precedence_point`, `occupancy_gate` など |
| `segment_id`                 | int         |  △ | 関連区間ID     |                                                                                |
| `s_event`                    | float       |  ○ | 全体アーク長位置   | 経路上の評価点                                                                        |
| `constraint_class`           | enum        |  ○ | 制約クラス      | `H`, `S1`, `S2`                                                                |
| `time_window_start`          | float/null  |  △ | 通過時刻窓開始    |                                                                                |
| `time_window_end`            | float/null  |  △ | 通過時刻窓終了    |                                                                                |
| `must_pass_before_event_id`  | string/null |  △ | 他イベントより前通過 | precedence                                                                     |
| `must_not_enter_before_time` | float/null  |  △ | この時刻以前は不可  |                                                                                |
| `occupancy_rule`             | enum/null   |  △ | 占有ルール      | `exclusive`, `shared`, `headway_required`                                      |
| `headway_time`               | float/null  |  △ | 最小時間間隔     |                                                                                |
| `penalty_weight`             | float/null  |  △ | 緩和時の重み     | soft 制約向け                                                                      |
| `coordination_region_id`     | string/null |  △ | 協調領域ID     |                                                                                |

---

## 3.4 Constraint class 定義

| クラス  | 意味                     | 例                                |
| ---- | ---------------------- | -------------------------------- |
| `H`  | Hard constraint        | 排他領域進入順序、絶対進入禁止、必須 precedence    |
| `S1` | Strong soft constraint | 強い time window、`v_min`、高優先区間の早通過 |
| `S2` | Weak soft constraint   | `v_nominal`、省エネ希望、停止回避希望         |

---

# 4. 空間⇒時間の受け渡し単位

1ロボット1経路について最低限次の情報を利用する。

| 名称                      | 中身         | 説明                    |
| ----------------------- | ---------- | --------------------- |
| `SpatialPathDescriptor` | 幾何の正本      |  `r(s)` を評価するための情報 |
| `IntentProfile`         | 時間・運用意図の正本 | retiming で尊重すべき情報 |
| `CapabilityAgreement`   | 任意         | 装置制約・両社合意制約の別紙      |

---

# 5. 時間軌道生成時の使い方イメージ

1. `SpatialPathDescriptor` から

   * 位置
   * 接線
   * 曲率
   * 区間境界
     を取得

2. `IntentProfile` から

   * 速度レンジ
   * time window
   * precedence
   * occupancy rule
     を取得

3. 装置制約

   * 最大速度
   * 最大加速度
   * jerk
   * 共振回避
   * 追従帯域
     を加味して最終 retiming

---

# 6. 初期最小セット

## SpatialPathDescriptor 最小セット

* `path_id`
* `robot_id`
* `frame_id`
* `total_length`
* `geometry_segments`

  * `segment_id`
  * `geometry_type`
  * `s_start`, `s_end`
  * line / quintic の幾何パラメータ
* `path_samples`

## IntentProfile 最小セット

* `path_id`
* `segment_level_intent`

  * `segment_id`
  * `segment_type`
  * `constraint_class`
  * `v_min / v_nominal / v_max`
  * `priority_local`
  * `stop_allowed`
  * `coordination_region_id`
* `event_level_intent`

  * `event_id`
  * `s_event`
  * `event_type`
  * `constraint_class`
  * `time_window_start / end`
  * `must_pass_before_event_id`

---

# 7. 責任境界との対応

| 領域                    | A社責任               | B社責任          |
| --------------------- | ------------------ | ------------- |
| SpatialPathDescriptor | 幾何妥当性、区分妥当性        | 正しく評価できること    |
| IntentProfile         | 意図妥当性、優先度、時間窓、協調意図 | 実現可能な範囲で満たすこと |
| 最終時間軌道                | 要求提示               | 実機制約下での実現     |
| 実現不能時返却               | 受理・再計画             | 理由付き返却        |

---


### SpatialPathDescriptor

> SpatialPathDescriptor は、A社が定義する空間経路の正本である。
> 本オブジェクトは、全体アーク長 `s` に沿った区分幾何経路を表し、各区間について位置、接線、曲率を評価可能な情報を含む。
> `line` 区間は `s` による閉形式、`quintic_corner` 区間は `u` による解析表現と `s(u)` テーブルを用いて評価される。

### IntentProfile

> IntentProfile は、上位の通過意図の正本である。
> 本オブジェクトは、区間ごとの速度レンジ、時間窓、優先度、停止可否、協調領域情報、およびイベントレベルの precedence / occupancy 制約を含む。
> 本 IntentProfile を hard / soft 制約として解釈し、装置制約を踏まえた最終 retiming を行う。


