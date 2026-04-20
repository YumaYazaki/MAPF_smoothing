**これまでの検討の流れを時系列順に整理した全体像**
「何を検証し、どんな結果が出て、それを踏まえて何を次にやったか」

---

# 1. 問題設定の固定

最初に、今回の対象問題を **姿勢付きMAPF** として固定しました。

* 状態は `(row, col, mode)`
* primitive は

  * `wait`
  * `move`
  * `rotate`
  * `move_rotate`
* 衝突判定は

  * `state occupancy`
  * `transition envelope`
* まずは **Level1 の姿勢付き古典 MAPF** を PoC 対象にする

この時点で、
「連続軌道化そのもの」ではなく、**まず離散の姿勢付き MAPF が解けるか** を検証軸に定めました。

---

# 2. 幾何モデルと可視化・衝突判定の整備

次に、ロボットの幾何を定義し、GUI と衝突判定が一致するように調整しました。

やったことは主に次です。

* ロボット形状を

  * ムーバー
  * アーム
  * 搬送物
    で定義
* 左右対称・前後非対称の形状へ修正
* 回転中心の描画ズレを修正
* `move+rotate` 時の envelope 計算を見直し
* 離散姿勢と連続回転の回転方向不整合を修正
* 可視化上で障害物接触に見えるが solver は通す、という不整合を解消

この段階で、
**GUI / 幾何 / 衝突判定の整合** をかなり丁寧に詰めました。
ここを飛ばさずに詰めたのは正しかったです。後の benchmark の信頼性がこれで担保されました。

---

# 3. Phase 1 baseline として CBS+A* を実装

その後、**Phase 1 baseline** として `CBS+A*` を実装しました。

やったこと:

* `cbs.py` を実装
* 低レベル探索は時間展開 A*
* 高レベルは CBS
* path signature 重複除去
* bypass 的な分岐抑制
* debug print の追加
* 高レベル探索が止まらない問題に対して

  * `max_wallclock_sec`
  * `max_high_level_expansions`
    を追加

つまり、まずは **きちんと benchmark を最後まで回せる CBS baseline** を作りました。

---

# 4. 最初の benchmark 群を作成

次に、基本 benchmark 群を設計しました。

* Case 01: rotation only
* Case 02: move+rotate
* Case 03: two robot corridor
* Case 04: obstacle wall
* Case A: move+rotate helpful
* Case B: move+rotate likely unhelpful
* Case C: coordination sensitive

この段階の狙いは、

* primitive の sanity check
* `move+rotate` の有効 / 不要の確認
* 単純協調からやや強い協調までの確認

でした。

---

# 5. Phase 1 benchmark 実行と初期結論

Phase 1 の結果から、まず次が確認できました。

* Case 02, 04, A, C では `move+rotate` が有効
* Case 03, B では `move+rotate` はほぼ不要
* Case C は quality は良いが、CBS+A* では非常に遅い
  `move+rotate=True` で約 46 秒かかる一方、`cost=14.2` まで詰められた 

ここから得た結論は、

> **問題設定としての `move+rotate` の価値はある。**
> ただし、CBS+A* は baseline としては有効でも、最終的な実用時間を目指す本命ではない。

でした。

---

# 6. Phase 2 として LaCAM 系 backend に進む方針を決定

この結果を踏まえて、次は **Phase 2** として LaCAM 系へ進む方針にしました。

ただし、いきなり本家 C++ 実装を深く改造するのではなく、

* 問題表現は自前で固定
* `OrientationGraph` を直接使う
* Python で **LaCAM 風 backend** をまず作る

方針にしました。

この時点で整理した責務分担は次です。

* **自前で固定する層**

  * `benchmark_common.py`
  * `geometry.py`
  * `environment.py`
  * `orientation_graph.py`
  * `lacam_adapter.py`
* **差し替え対象**

  * backend / 探索器

つまり、**問題表現は固定し、探索器だけを変える**構造を作りました。

---

# 7. Phase 2 の基盤ファイルを整備

そこで以下を順に整備しました。

* `benchmark_common.py`
* `orientation_graph.py`
* `lacam_adapter.py`
* `lacam_runner.py`
* `phase2_benchmark_lacam.py`

この段階では、まずは `not_implemented` でもよいから
**Phase 2 の benchmark 配線だけでも完成させる** ことを重視しました。

---

# 8. PythonOrientationLaCAMBackend の最初の実装（step1）

次に、`PythonOrientationLaCAMBackend` を実装しました。

この最初の版の性格は、

* 各ロボットを単独で解く
* conflict を見つける
* conflicted robot を局所再計画する

という、かなり **Prioritized Planning + local repair** に近いものでした。

この時点での狙いは、
**LaCAM* 完全再現ではなく、まず Phase 2 を回し始めること** でした。

---

# 9. benchmark が足りないと判断し、Tier 4 を追加

その後、「Case C だけでは backend 改善の差分が見えにくい」と判断し、Tier 4 の難ケースを追加しました。

* Case D: 3-agent single bottleneck
* Case E: 4-agent cross intersection
* Case F: pocket evacuation
* Case G: double bottleneck
* Case H: asymmetric orientation-critical

この追加はかなり重要でした。
なぜなら、この追加で初めて

* backend の頑健性
* 初期解生成の弱さ
* repair の限界

が分かるようになったからです。

---

# 10. Phase 1 baseline を Tier 4 まで拡張して再評価

Tier 4 追加後に CBS+A* baseline を回した結果、

* Case D: timeout
* Case E: fail
* Case G: fail
* Case H: fail
* Case F: 解けるが `move+rotate=True` でも大きな改善は出ない

という結果になりました。
つまり、**CBS+A* は Tier 4 に入るとかなり厳しい**と分かりました。 

この結果で、
Phase 2 の backend 比較に進む意味がはっきりしました。

---

# 11. Phase 2 step2: repair 候補比較を導入

次に、最初の Python backend の弱点だった

* 最初に見つかった repair を即採用
* 候補比較がない

部分を改修しました。

具体的には、

* `RepairCandidate`
* `_count_conflicts()`
* `_evaluate_repair_candidate()`
* `_generate_single_agent_repair_candidates()`
* `_generate_two_agent_repair_candidates()`
* `_select_best_repair_candidate()`

を追加して、
**候補生成 → 候補評価 → 候補採用** の形にしました。

---

# 12. Phase 2 step2 の結果と解釈

この改修後の benchmark では、

* Case C が大きく改善
* Case D/F/G の一部を短時間で解ける
* Case E は `initial_planning_failed`
* Case G/H は `repair_failed`

という結果が出ました。
特に Case C は `move+rotate=True` で `makespan=9.2`, `cost=13.2`、しかも約 0.11 秒で解けており、CBS+A* より速くて quality も良い状態になりました。 

ここで重要だったのは、失敗モードが分離できたことです。

* **Case E**
  → 初期解生成が弱い
* **Case G/H**
  → repair が弱い

つまり、

> **初期解生成改善** と **repair 拡張** を分けて考えるべき

という判断に至りました。

---

# 13. 「repair をさらに詰めるべきか」を議論

その後、
「ここで repair を詰めても、最終的に LaCAM ベースを目指すなら無駄ではないか？」
という論点が出ました。

その結果、次のように整理しました。

* **無駄にならない repair 改善**

  * backend 非依存の知見になるもの
  * conflict 評価
  * 候補比較
  * score 設計
* **無駄になりやすい repair 改善**

  * 今の暫定 backend 専用の hack
  * 局所実装都合だけの最適化

この整理のうえで、
「repair はもう1段だけ詰めてもよいが、やりすぎない」という方針にしました。

---

# 14. 初期解生成改善が次の論点だと判断

ただ、Tier 4 結果を見直すと、Case E は **repair の前に失敗** しているので、
次の優先テーマは **初期解生成改善** だと判断しました。

具体的には、

> **planning order を変えるだけでは足りない**

という結論になりました。

Case E の 4-way intersection や、Case H の姿勢依存ケースでは、

* 誰を先に解くか
  だけでなく、
* 誰を少し待たせるか
* reservation を hard forbid ではなく soft に扱うか

が重要だと考えました。

---

# 15. soft reservation + 初期遅延付き初期化の設計

そこで、Case E/H 向けの初期解生成改善案として、

> **soft reservation + 初期遅延候補付き初期化**

を設計しました。

追加 / 変更対象として整理した関数は次です。

* `_generate_initial_orders()`
* `_generate_initial_delay_candidates()`
* `_plan_single_agent_path_soft_reservation()`
* `_compute_soft_reservation_penalty()`
* `_build_initial_solution_for_order_and_delay()`
* `_evaluate_initial_solution_candidate()`
* `_select_best_initial_solution()`

狙いは、

* reservation を hard forbid ではなく penalty 化
* 各 robot に `0,1,2,3` ステップ程度の初期遅延候補を与える
* order × delay の組合せから最良の seed を選ぶ

ことでした。

---

# 16. そこまで踏まえて現在地

現時点での全体の流れを一言でまとめると、こうです。

1. **姿勢付き MAPF の問題設定を固定**
2. **CBS+A* で baseline を作成**
3. **基本 benchmark で `move+rotate` の有効性を確認**
4. **Tier 4 を追加して baseline の限界を確認**
5. **PythonOrientationLaCAMBackend を作り、Phase 2 を開始**
6. **repair 候補比較を導入して Case C/D/F/G の一部を改善**
7. **Case E は初期解生成、Case G/H は repair がボトルネックだと分離**
8. **その結果、次の論点は soft reservation + 初期遅延付き初期化になった**

---

# 17. 次にやるべきこと

この流れを踏まえると、次チャット / 次フェーズでやるべきことは明確です。

## 第一候補

**soft reservation + 初期遅延対応版 `lacam_backends.py` を実際に benchmark して、Case E/H がどう変わるかを見る**

確認対象:

* Case E
* Case H
* Case G
* 既存の Case C / D / F が悪化していないか

## その次

結果に応じて、

* **Case E/H が初期化通過**
  → repair 拡張へ
* **まだ初期化 fail**
  → penalty / delay / order 候補の再設計へ
