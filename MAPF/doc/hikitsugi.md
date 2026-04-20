# 引継ぎ資料：姿勢付きMAPF + LaCAM系 backend 検討

## 1. プロジェクトの目的

目的は、**姿勢付き MAPF** を対象として、

* Phase 1: **CBS+A*** を baseline として benchmark を固定
* Phase 2: **LaCAM 系 backend** を導入し、比較可能な形で評価
* 最終的には **実用時間で動く backend** に持っていく

こと。

対象問題の特徴は次の通り。

* 状態: `(row, col, mode)`
* primitive:

  * `wait`
  * `move`
  * `rotate`
  * `move_rotate`
* 衝突判定:

  * `state occupancy`
  * `transition envelope`
* `move+rotate` の有効性を benchmark で比較したい

---

## 2. これまでの設計上の重要判断

### 2.1 問題表現は自前で固定する

以下は **自前実装・固定対象** とした。

* `benchmark_common.py`
* `geometry.py`
* `environment.py`
* `orientation_graph.py`
* `lacam_adapter.py`

つまり、**問題定義は固定し、探索器だけ差し替える**方針。

### 2.2 LaCAM の扱い

方針としては、まず

* `OrientationGraph` を直接使う
* `PythonOrientationLaCAMBackend` を作る

方を採用。

理由:

* いきなり本家 C++ LaCAM に深く入らずに Phase 2 を回せる
* benchmark 比較基盤を早く固められる
* 後で C++ / 本家寄り backend に差し替えやすい

### 2.3 backend の現状の位置づけ

現状の `PythonOrientationLaCAMBackend` は、
厳密には **LaCAM* 完全再現ではなく**、

> Prioritized Planning + local repair + candidate comparison

に近い軽量 backend。

ただし、Phase 2 の比較開始には十分有効。

---

## 3. 現在のファイル構成の考え方

### 3.1 benchmark / adapter 系

* `benchmark_common.py`

  * `BenchmarkCaseSpec`
  * `BenchmarkResult`
  * save/load
  * environment 構築
* `orientation_graph.py`

  * `OrientedState`
  * `OrientedEdge`
  * `OrientationGraph`
  * `build_oriented_problem()`
* `lacam_adapter.py`

  * `LaCAMProblem`
  * `LaCAMSolution`
  * `build_lacam_problem()`
  * `lacam_solution_to_robot_paths()`

### 3.2 runner / backend 系

* `lacam_runner.py`

  * `LaCAMRunner`
  * `LaCAMRunnerConfig`
  * `run_lacam()`
* `lacam_backends.py`

  * `PythonOrientationLaCAMBackend`
  * 現状は

    * repair 候補比較
    * planning order 候補
    * soft reservation + 初期遅延案
      まで検討・コード生成済み

### 3.3 CBS baseline 系

* `cbs.py`

  * `max_wallclock_sec`
  * `max_high_level_expansions`
    を追加済み
* `phase1_benchmark_cbs.py`

  * 上記引数を `cbs_plan()` に渡す版を作成済み

---

## 4. benchmark 設計

### 4.1 基本ケース

* Case 01: rotation only
* Case 02: move+rotate
* Case 03: two robot corridor
* Case 04: obstacle wall
* Case A: move+rotate helpful
* Case B: move+rotate likely unhelpful
* Case C: coordination sensitive

### 4.2 Tier 4 追加ケース

追加済みの難ケース:

* Case D: 3-agent single bottleneck
* Case E: 4-agent cross intersection
* Case F: pocket evacuation
* Case G: double bottleneck
* Case H: asymmetric orientation-critical

この Tier 4 追加がかなり重要だった。
理由は、Case C だけでは backend 改善が Case C 専用チューニングになる恐れがあったため。

---

## 5. Phase 1 / Phase 2 の benchmark 結果の要点

添付結果から見えるポイントを整理すると、

### 5.1 Phase 1 (CBS+A*)

* 基本ケースは解ける
* ただし Tier 4 で急激に厳しくなる
* Case D は timeout
* Case E, G, H は失敗
* Case F は解けるが `move+rotate=True` でも大きな改善は出ない
* Case C は quality は高いが遅い
  例: `move+rotate=True` で約 46 秒、`cost=14.2` 

### 5.2 Phase 2 (PythonOrientationLaCAMBackend, step2 時点)

* 基本ケースは問題なく解ける
* Case C は非常に速く、かつ良好な quality
  `move+rotate=True` で `makespan=9.2`, `cost=13.2`, 約 0.11 秒 
* Case D は解ける
* Case F も解ける
* Case G は `move+rotate=True` で解ける
* Case E は `initial_planning_failed`
* Case H は `repair_failed`
  つまり、

  * E は **初期解生成がボトルネック**
  * G/H は **repair スコープがボトルネック**
    と解釈した 

---

## 6. ここまでの重要な解釈

### 6.1 `move+rotate` の有効 / 不要はかなり一貫

基本ケースでは、

* 02, 04, A, C, D, F, G(true) などで有効
* 03, B では不要

という整理は Phase 1 / 2 を通して大きく崩れていない。
つまり、これは **探索器依存ではなく問題構造側の本質** と見てよい。 

### 6.2 Phase 2 backend はすでに有望

現状の Python backend はまだ暫定だが、
少なくとも Tier 4 の一部では CBS+A* より

* 速い
* 実行可能
* 場合によっては quality も高い

という結果が出ている。
特に Case C/D/F/G がその根拠。 

### 6.3 失敗モードの分離ができた

これが今回の benchmark 追加の最大成果。

* `initial_planning_failed`
  → 初期解生成改善が必要
* `repair_failed`
  → repair 拡張が必要

この分離により、次の設計判断がしやすくなった。 

---

## 7. ここまでに到達した設計判断

### 7.1 repair を詰めるのは無駄か？

結論:

* **backend 非依存の知見として残る改善**なら無駄ではない
* ただし暫定 backend 専用の hack はやりすぎない

### 7.2 追加 benchmark の後の判断

Tier 4 追加後は、
repair をさらに詰める前に benchmark の複雑さ不足は概ね解消したと見てよい。

### 7.3 現在の優先テーマ

* Case E/H を解けるようにするための **初期解生成改善**
* Case G/H を解けるようにするための **repair 拡張**

---

## 8. 現在の最新検討テーマ

### 8.1 Case E/H 向け初期解生成改善

方針:

> **soft reservation + 初期遅延候補付き初期化**

なぜ必要か:

* Case E/H は repair 前に失敗しているから
* 単なる planning order の工夫だけでは足りないから

### 8.2 具体案

初期解生成を次のように変える設計まで検討済み。

* `_generate_initial_orders()`
* `_generate_initial_delay_candidates()`
* `_plan_single_agent_path_soft_reservation()`
* `_compute_soft_reservation_penalty()`
* `_build_initial_solution_for_order_and_delay()`
* `_evaluate_initial_solution_candidate()`
* `_select_best_initial_solution()`

要点:

* reservation を hard forbid ではなく penalty 化
* 各 robot に delay 候補 `(0,1,2,3)` を持たせる
* 複数 order × 複数 delay の候補から最良を選ぶ

### 8.3 そのコード

`lacam_backends.py` の **soft reservation + 初期遅延対応版全文** まで生成済み。
ただし、その版を実行した benchmark 結果は、現時点の添付結果にはまだ反映されていない可能性があるため、**次チャットでまずその版を実行し、Case E/H がどう変わるか確認する**のが重要。

---

## 9. 次チャットで最初にやるべきこと

次チャットでは、以下の順で進めるのが自然。

### Step 1

現在の `lacam_backends.py` が

* soft reservation + 初期遅延対応版
  になっているか確認

### Step 2

`phase2_benchmark_lacam.py` を再実行し、特に以下を見る

* Case E
* Case H
* Case G
* 既存の Case C / D / F が悪化していないか

### Step 3

結果に応じて分岐

* **Case E/H が初期化通過**
  → 次は repair 側の拡張へ
* **まだ `initial_planning_failed`**
  → soft reservation penalty / delay 候補 / order 候補を再調整

---

## 10. 次チャットで相談したい論点

### 論点A

soft reservation + 初期遅延で Case E/H が通るか

### 論点B

Case G/H に対して、

* pairwise repair のままでよいか
* 3-agent 近傍 repair や beam 的候補管理が必要か

### 論点C

将来的な LaCAM 本命化を見据えて、

* どこまで Python backend で知見を取るか
* どの時点で C++ / 本家寄りへ移るか

---

## 11. 次チャットの最初の一文（推奨）

次チャットの冒頭には、これを貼るとスムーズです。

```text
姿勢付きMAPF + LaCAM系 backend の続きです。

現状:
- Phase 1: CBS+A* baseline は完成
- Phase 2: PythonOrientationLaCAMBackend で benchmark 比較中
- Tier 4 ケース D/E/F/G/H を追加済み
- E は initial_planning_failed、G/H は initial/repair がボトルネック
- 最新テーマは Case E/H 向けの soft reservation + 初期遅延付き初期化

今回は、soft reservation + 初期遅延対応版 lacam_backends.py を前提に、
Case E/H がどう変わるかを確認し、その次の改修方針を決めたいです。
```
