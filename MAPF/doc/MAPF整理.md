はい。
MAPF（Multi-Agent Path Finding）は名前の多い分野ですが、整理の軸を決めるとかなり見通しが良くなります。

いちばん実務で使いやすい整理は、次の4軸です。
**①何を最適化するか、②どう競合を解くか、③時間・幾何をどこまで表現するか、④どこまで保証するか** です。MAPFの古典的定義や代表的ベンチマーク整理は、SoCS 2019の定義論文が基準として使いやすいです。 ([arXiv][1])

---

## 1. まず、MAPFとは何か

古典的MAPFは、グラフ上で複数エージェントをそれぞれの始点から終点へ、**衝突なし**で同時に移動させる問題です。代表的な衝突は、同じ頂点への同時到達と、同じ辺のすれ違いです。目的関数としては主に **makespan**（全員完了までの時間）と **sum of costs / flowtime**（各エージェントの到達時間総和）が使われます。 ([arXiv][1])

ただし実際の研究・実装では、古典的MAPFからかなり広がっていて、

* 連続時間
* 非点形状
* 動力学制約
* ライフロング（タスクが流れ続ける）
* 遅延に対するロバスト性
* 学習併用
* 分散実行

まで含めることが多いです。 ([IJCAI][2])

---

## 2. 大分類の全体像

MAPFアルゴリズムは、大きく次の7群に分けると整理しやすいです。

### A. 結合探索型

全エージェントをひとつの巨大状態としてまとめて探索します。
古典的には joint-state A* 系です。最も素直ですが、状態爆発しやすいです。MAPFの難しさの基準点として重要です。 ([arXiv][1])

### B. 分離型・競合解消型

各エージェントは基本別々に計画し、衝突したときだけ制約を追加して解消します。
代表は **CBS** とその派生です。現在のMAPFで最も中心的な系統の一つです。 ([AAAI][3])

### C. コスト分解型

各エージェントのコスト割当を先に決め、その範囲で整合解があるかを調べます。
代表は **ICTS** です。 ([IJCAI][4])

### D. 優先度ベース型

優先順位を決めて、上位から順に計画します。
古典的には **Cooperative A***、**HCA***、**WHCA***、最近も優先度法は高速性のため重要です。最適性は通常ありませんが、速いです。 ([AAAI][5])

### E. 局所結合・部分次元拡大型

通常は分離して動き、干渉する集団だけ局所的に結合して探索します。
代表は **M***（subdimensional expansion）です。 ([Kilthub][6])

### F. コンパイル型

MAPFを SAT / ILP / CSP / ASP などへ変換して汎用ソルバで解きます。
理論的に強く、最適解や厳密解に向きますが、実時間用途では選び方が重要です。MAPFのコンパイル系は2022年のサーベイがまとまっています。 ([IJCAI][7])

### G. 学習・ヒューリスティック・LNS型

学習そのもので解くものもありますが、実務上は **探索を速くする補助** として使われることが多いです。近年は anytime 系の **Large Neighborhood Search (LNS)** や、CBS の競合選択を学習する手法も目立ちます。 ([arXiv][8])

---

## 3. 代表アルゴリズムを系統ごとに整理

### 3.1 結合探索型

#### Joint A*

全エージェント状態をまとめて A* します。
完全・最適化可能ですが、状態空間は各エージェントの直積なので、台数が増えると急速に厳しくなります。MAPF定義論文でも、これが基礎的だがスケールしにくい見方として扱われます。 ([arXiv][1])

#### Operator Decomposition

結合探索の1ステップで全員同時に動かす代わりに、部分的に遷移させて分岐を減らす系統です。古典的な高速化の一つですが、近年はCBS系に比べて主役ではありません。MAPFの古典整理でよく登場します。 ([arXiv][1])

**向き不向き**
台数が少なく、厳密最適性が必要で、問題規模が大きくないなら理解しやすいです。
ただし 4〜6台でも環境が混むと急速に厳しくなります。 ([arXiv][1])

---

### 3.2 競合解消型：CBS系

#### CBS

CBS は **高レベルで競合を選んで制約分岐し、低レベルで各エージェントを単独再計画**する二層法です。最適・完全で、古典MAPFの代表的アルゴリズムです。 ([AAAI][3])

#### ICBS

CBS に対して、競合選択・バイパス・メタエージェント化などの改善を入れたものです。CBSの実用性を大きく押し上げた系統です。 ([IJCAI][9])

#### Disjoint Splitting

CBS の分岐方法を改善し、探索木の重複を減らす方法です。CBS派生ではかなり重要です。 ([people.eng.unimelb.edu.au][10])

#### MA-CBS

繰り返し衝突するエージェントを束ねてひとつのメタエージェントとして扱う拡張です。局所的に結合探索へ寄せる発想です。 ([WebDocs][11])

#### ECBS / EECBS

CBSの**bounded-suboptimal**版です。最適性を少し緩めて高速化します。
EECBS は Explicit Estimation Search を組み合わせて、ECBSより高速な bounded-suboptimal 解法として知られています。実務では「100ms級で何とかしたい」場合に、最適CBSよりこちらの方が現実的なことが多いです。 ([AAAI][12])

#### CCBS

**Continuous-time CBS** です。時間を離散化せず、連続時間で扱います。非一様時間や実形状に近い扱いがしやすく、SIPP を低レベルに使うのが特徴です。古典CBSの「離散時間」仮定を外した重要な拡張です。 ([IJCAI][2])

**CBS系の特徴**
いまのMAPFで、最も「中心的な標準解法群」のひとつです。最適性・説明性・制約追加による拡張性が強い一方、高密度・多台数では高レベル木が膨らみやすいです。 ([AAAI][3])

---

### 3.3 コスト分解型：ICTS系

#### ICTS

高レベルで各エージェントのコストベクトルを探索し、低レベルでそのコストに収まる衝突なし経路が存在するかを調べます。最適です。CBSと並ぶ古典的な二層法です。 ([IJCAI][4])

#### Extended ICTS

非単位コストや非単位時間へ拡張したものです。古典ICTSの前提を現実寄りに広げています。 ([IJCAI][13])

#### Bounded-suboptimal ICTS

最適性を少し緩めたICTS系もありますが、近年の主流感はCBS系ほど強くありません。 ([WebDocs][14])

**ICTS系の特徴**
コスト構造が明快で、問題によっては非常に強いです。ただし実務での拡張性や最近の主流感はCBS派生の方が強い印象です。 ([IJCAI][4])

---

### 3.4 優先度ベース型：PP / CA* / HCA* / WHCA* 系

#### Prioritized Planning

エージェントに優先順位をつけて、先に計画した経路を以後の動的障害物として扱います。高速ですが、通常は完全でも最適でもありません。 ([AAAI][5])

#### Cooperative A*

Silver の Cooperative Pathfinding は、予約テーブルを使って複数エージェントの衝突を避ける古典的枠組みで、CA*、HCA*、WHCA* などの土台です。 ([AAAI][5])

#### HCA* / WHCA*

空間抽象や時間窓を使って高速化する系統です。WHCA* はオンライン・リアルタイム寄りでよく言及されます。 ([AAAI][5])

#### COWHCA*

Conflict-Oriented Windowed Hierarchical Cooperative A* は、WHCA* 系の競合重視改良です。ウィンドウ付きで速く動かしたい場合の代表例です。 ([ベン＝グリオン大学][15])

**優先度系の特徴**
速くて実装しやすく、ライフロングや大規模でも使いやすい一方、順序依存が強く、最適性・完全性は弱いです。リアルタイム用途では依然として非常に重要です。 ([AAAI][5])

---

### 3.5 部分次元拡大型：M*系

#### M*

通常は各エージェントを独立に扱い、衝突が起きる部分だけ局所的に結合して探索空間を広げる手法です。**subdimensional expansion** の代表例です。 ([Kilthub][6])

#### 派生

M* に bypass や multi-objective を入れた派生もあります。近年でもこの思想は活きています。 ([ResearchGate][16])

**M*系の特徴**
「全結合探索は重すぎるが、完全分離も無理」という中間に強いです。衝突が局所的ならかなり魅力があります。 ([Kilthub][6])

---

### 3.6 コンパイル型：SAT / ILP / CSP / ASP

MAPFを命題充足や整数計画へ落として解く方法です。
2022年の survey は、SAT系・MDD-SAT系・CSP系などの整理に有用です。 ([IJCAI][7])

#### SAT-based

MAPFを時間展開グラフ上の SAT に変換します。最適化は makespan や flowtime の境界を増やしながら判定する形が多いです。 ([IJCAI][7])

#### ILP / MILP

制約が多い、コスト表現が柔軟、物流的な制約を入れやすい、という利点があります。ただし実時間用途では問題サイズ次第です。 ([IJCAI][7])

#### CP / CSP / ASP

論理制約や追加ルールが多い場合に相性が良いです。説明性も比較的高いです。 ([IJCAI][7])

**コンパイル型の特徴**
拡張制約を入れやすく、厳密性が高い一方、100ms級オンライン再計画ではそのままだと厳しいことが多いです。オフライン検証や厳密ベンチマーク向きです。 ([IJCAI][7])

---

### 3.7 LNS・局所改善・anytime系

#### LNS-based MAPF

初期解を作ってから、一部エージェント群だけ抜き出して改善を繰り返します。近年、**柔軟性・スケーラビリティ・anytime性** で注目されています。2024年にもLNS再評価論文が出ています。 ([arXiv][8])

#### 局所探索・SLS

SAT変換後に局所探索する方法も研究されています。最適保証より大規模高速化が狙いです。 ([SciTePress][17])

**特徴**
初期解さえ出れば改善できるので anytime 性が高いです。最適保証は弱いですが、実務ではかなり魅力的です。 ([arXiv][8])

---

## 4. 重要な拡張テーマ別の整理

### 4.1 Continuous-time MAPF

古典MAPFは時間離散ですが、実機では連続時間が自然です。
代表は **CCBS**、低レベルで **SIPP** を使う系統です。連続時間、非単位時間、実形状に近いモデルへ広げやすいです。 ([IJCAI][2])

### 4.2 Lifelong MAPF

ゴール到達後も新しいタスクが流れ続ける設定です。倉庫や搬送で重要です。近年の研究は、より現実的な lifelong setting へのスケーリングが大きなテーマです。 ([arXiv][18])

### 4.3 Robust MAPF

実行遅れがあっても壊れにくい計画を求めます。
**k-robust MAPF** は、限られた遅延に耐える計画を扱う代表的定式化です。 ([ifaamas.org][19])

### 4.4 TAPF / Combined Assignment and Path Finding

目標割当と経路計画を同時に解く系統です。あなたの「どの組が行くか」に近い論点は、MAPF本体というよりこちらに近い場合があります。近年も ITA-CBS / ITA-ECBS のような派生があります。 ([mapf.info][20])

### 4.5 Continuous-time / volumetric lifelong

近年は、連続時間・容積を持つエージェント・ライフロングを同時に扱う方向も出ています。まだ研究途上ですが、実機に近いです。 ([ResearchGate][21])

---

## 5. 学習ベースMAPFの位置づけ

学習系は大きく3つに分けると分かりやすいです。

### 5.1 直接ポリシー学習

RLや模倣学習で、エージェントが直接動作を決めます。
分散型・大規模型では魅力がありますが、厳密な安全保証や説明性は弱くなりやすいです。 ([grafft.github.io][22])

### 5.2 探索支援学習

CBS の競合解消順や LNS の近傍選択など、既存アルゴリズムの中の意思決定だけを学習します。これは実務的にかなり有望です。 ([IDM Lab][23])

### 5.3 スケーリング支援

大規模ライフロングで imitation learning を使う方向も進んでいます。 ([arXiv][24])

**実務的評価**
工場・搬送系のように安全と説明性が強い場合、学習を主役にするより、**CBS / PP / LNS の補助**にする方が現実的なことが多いです。これは最近の文献の流れとも整合します。 ([IDM Lab][23])

---

## 6. 「網羅的」に見たときの実務分類

理論分類とは別に、実務では次のように選ぶのが分かりやすいです。

### 厳密最適が欲しい

* CBS
* ICBS
* ICTS
* CCBS
* SAT/ILP系
  この群は説明性が高く、PoCで「成立性の上限」を見るのに向きます。 ([AAAI][3])

### 100ms級や bounded-suboptimal が欲しい

* ECBS / EECBS
* WHCA* / COWHCA*
* 優先度法
* LNS系
  この群は「最適性を少し捨てて速さを取る」方向です。 ([AAAI][12])

### 連続時間・実形状・実機寄りが欲しい

* CCBS
* SIPP系
* Prioritized SIPP / PSIPP 系
  離散時間グリッドでは厳しいときに有力です。 ([IJCAI][2])

### ライフロング・大規模スループット重視

* Prioritized Planning 系
* RHCR系や rolling-horizon 系
* 学習補助付き大規模法
  近年の課題設定はここへ向かっています。 ([arXiv][18])

### 超制約付き・追加条件が多い

* ILP / SAT / CP
* ルールベースとのハイブリッド
  複雑なインターロックや工程制約を入れやすいです。 ([IJCAI][7])

---

## 7. どのアルゴリズムが「主流」か

2026年時点で、MAPFの中心はかなり明確で、

* **最適系の中核**: CBS とその改良群、ICTS、CCBS ([AAAI][3])
* **bounded-suboptimal / 実用系の中核**: ECBS / EECBS、優先度法、LNS 系 ([AAAI][12])
* **現実的拡張の中核テーマ**: continuous-time、lifelong、robust、learning-assisted ([IJCAI][2])

という見方でだいたい合っています。

---

## 8. あなたの文脈に引きつけると

あなたが扱っているのは、単純な古典MAPFではなく、

* 2D
* 2〜6台
* 角度や実形状が重要
* 連続軌道へ接続したい
* 100ms級
* 説明性が欲しい
* 緊急停止安全も気になる

という条件でした。
この条件では、理論的に「網羅的に全部候補」ではあっても、実際に主候補になるのはかなり絞れます。

率直に言うと、主候補は次です。

* **CBS/ECBS/EECBS 系**
  説明性・制約追加・固定軌道との整合が良い。 ([AAAI][3])
* **CCBS / SIPP系**
  連続時間・実形状寄りにしやすい。 ([IJCAI][2])
* **Prioritized / Windowed 系**
  100ms級や rolling-horizon に強い。 ([AAAI][5])
* **LNS / 学習補助**
  anytime 性や実用高速化の補助として有望。 ([arXiv][8])

逆に、純粋 joint-state A* やコンパイル型をそのまま本番中核に置くのは、PoCやベンチ用途ならともかく、リアルタイム実装では優先度が下がりやすいです。 ([IJCAI][7])

---

## 9. 一枚で見る要約

### 最適・説明しやすい

CBS, ICBS, ICTS, CCBS, SAT/ILP ([AAAI][3])

### 速い・現実的

Prioritized Planning, WHCA*, ECBS/EECBS, LNS ([AAAI][5])

### 干渉局所化が効く

M* / subdimensional expansion ([Kilthub][6])

### 実機寄り

CCBS, SIPP系, robust MAPF, lifelong MAPF ([IJCAI][2])

### 学習は補助が本命

CBS支援学習、LNS支援学習、模倣学習によるライフロング支援 ([IDM Lab][23])

---

必要なら次に、
**「あなたの要件に対して、各MAPFアルゴリズムを適合度順に並べた選定表」** まで落とします。

[1]: https://arxiv.org/pdf/1906.08291?utm_source=chatgpt.com "Multi-Agent Pathfinding: Definitions, Variants, and ..."
[2]: https://www.ijcai.org/proceedings/2019/0006.pdf?utm_source=chatgpt.com "Multi-Agent Pathfinding with Continuous Time"
[3]: https://cdn.aaai.org/ojs/8140/8140-13-11667-1-2-20201228.pdf?utm_source=chatgpt.com "Conflict-Based Search For Optimal Multi-Agent Path Finding"
[4]: https://www.ijcai.org/Proceedings/11/Papers/117.pdf?utm_source=chatgpt.com "The Increasing Cost Tree Search for Optimal Multi-Agent ..."
[5]: https://cdn.aaai.org/ojs/18726/18726-52-22369-1-10-20210928.pdf?utm_source=chatgpt.com "Cooperative Pathfinding"
[6]: https://kilthub.cmu.edu/articles/Subdimensional_Expansion_for_Multirobot_Path_Planning/6561017/files/12043325.pdf?utm_source=chatgpt.com "Subdimensional Expansion for Multirobot Path Planning"
[7]: https://www.ijcai.org/proceedings/2022/0783.pdf?utm_source=chatgpt.com "Problem Compilation for Multi-Agent Path Finding: a Survey"
[8]: https://arxiv.org/abs/2407.09451?utm_source=chatgpt.com "Reevaluation of Large Neighborhood Search for MAPF"
[9]: https://www.ijcai.org/Proceedings/15/Papers/110.pdf?utm_source=chatgpt.com "ICBS: Improved Conflict-Based Search Algorithm for Multi- ..."
[10]: https://people.eng.unimelb.edu.au/pstuckey/papers/icaps19a.pdf?utm_source=chatgpt.com "Disjoint Splitting for Multi-Agent Path Finding with Conflict- ..."
[11]: https://webdocs.cs.ualberta.ca/~nathanst/papers/sharon2012macbs.pdf?utm_source=chatgpt.com "Meta-agent Conflict-Based Search For Optimal Multi- ..."
[12]: https://cdn.aaai.org/ojs/17466/17466-13-20960-1-2-20210518.pdf?utm_source=chatgpt.com "A Bounded-Suboptimal Search for Multi-Agent Path Finding"
[13]: https://www.ijcai.org/proceedings/2018/0074.pdf?utm_source=chatgpt.com "Extended Increasing Cost Tree Search for Non-Unit ..."
[14]: https://webdocs.cs.ualberta.ca/~nathanst/papers/ICT.pdf?utm_source=chatgpt.com "Finding Bounded Suboptimal Multi-Agent Path Planning ..."
[15]: https://tzin.bgu.ac.il/~felner/2014/COWA6p.pdf?utm_source=chatgpt.com "Conflict-Oriented Windowed Hierarchical Cooperative A*"
[16]: https://www.researchgate.net/publication/362385509_Multi-Agent_Path_Finding_Based_on_Subdimensional_Expansion_with_Bypass/download?utm_source=chatgpt.com "(PDF) Multi-Agent Path Finding Based on Subdimensional ..."
[17]: https://www.scitepress.org/Papers/2024/129448/129448.pdf?utm_source=chatgpt.com "Solving Multi-Agent Pathfinding with Stochastic Local ..."
[18]: https://arxiv.org/pdf/2404.16162?utm_source=chatgpt.com "arXiv:2404.16162v1 [cs.MA] 24 Apr 2024"
[19]: https://ifaamas.org/Proceedings/aamas2018/pdfs/p1862.pdf?utm_source=chatgpt.com "Robust Multi-Agent Path Finding"
[20]: https://mapf.info/index.php/Main.SKoen24l?utm_source=chatgpt.com "Main / Publication"
[21]: https://www.researchgate.net/publication/389946253_Prioritized_Planning_for_Continuous-time_Lifelong_Multi-agent_Pathfinding?utm_source=chatgpt.com "Prioritized Planning for Continuous-time Lifelong Multi- ..."
[22]: https://grafft.github.io/assets/pdf/follower2024.pdf?utm_source=chatgpt.com "Learn to Follow: Decentralized Lifelong Multi-Agent ..."
[23]: https://idm-lab.org/bib/abstracts/papers/aaai21c.pdf?utm_source=chatgpt.com "Learning to Resolve Conflicts for Multi-Agent Path Finding ..."
[24]: https://arxiv.org/pdf/2410.21415?utm_source=chatgpt.com "Scalable Imitation Learning for Lifelong Multi-Agent Path ..."



はい、**LaCAM / LaCAM*** は候補に入れるべきです。
特に、前に整理したMAPF系の中では、CBS系やICTS系とは少し違う **「構成(configuration)ベースで、まず速く解を見つけ、その後に改善しやすい系統」** として位置づけるのが分かりやすいです。LaCAM は AAAI 2023 で提案された **complete** なMAPF解法で、LaCAM* は IJCAI 2023 で提案された **eventually optimal / anytime** 指向の改良版です。IJCAI 2023 の論文では、標準的なデスクトップPCで多数エージェントに対して10秒以内に解を見つけつつ、最終的に最適へ収束する方向を狙っていることが明記されています。 ([AAAI Digital Library][1])

整理し直すと、MAPFの候補群はこう見るのが自然です。
**厳密最適の本流** は CBS / ICBS / ICTS / CCBS、**実用高速・bounded-suboptimal** は ECBS / EECBS / Prioritized 系 / WHCA* 系、そして **「まず速く有効解を出し、その後改善していく anytime・大規模寄りの探索系」** に LaCAM / LaCAM* を置く、という形です。LaCAM は「two-level search」で高レベルでは全エージェントの configuration を探索し、低レベルでは次 configuration を満たす制約を生成する方式として説明されています。 ([AAAI Digital Library][1])

あなたの文脈に引きつけると、LaCAM* を入れる意味はかなりあります。
理由は、あなたが重視していたのが **100ms級のリアルタイム性、anytime性、説明性、2〜6台程度、ただし混雑時も見たい** という条件だったからです。LaCAM* は「まず解を速く見つける」「時間が許せば改善する」という方向性なので、**PoCで“制限時間内にどの程度の品質が出るか”を評価する候補**としてかなり相性が良いです。一方で、古典的CBSのような「競合を1つずつ論理的に分岐して解く」タイプに比べると、説明の仕方は少し変わります。CBSは「この衝突を避けるためにこの制約を足した」と言いやすいのに対し、LaCAM系は **configuration 探索の誘導や改善過程** を説明することになります。 ([AAAI Digital Library][1])

なので、前の候補整理を更新すると、実務上の主な候補は次のように見直すのがよいです。

| 系統                    | 代表手法                            | 向いていること                     | 注意点                   |
| --------------------- | ------------------------------- | --------------------------- | --------------------- |
| 厳密最適・説明性重視            | CBS, ICBS, ICTS, CCBS           | 制約追加の説明がしやすい。PoCで成立性上限を見やすい | 混雑・多台数・短時間制約では重くなりやすい |
| 高速 bounded-suboptimal | ECBS, EECBS, WHCA*, Prioritized | 100ms級やrolling horizon寄り    | 最適性は緩める必要がある          |
| anytime・大規模寄り探索       | **LaCAM, LaCAM***               | まず速く解を出し、時間があれば改善する         | CBSほどの「衝突ごとの説明」はしにくい  |
| 連続時間・実機寄り             | CCBS, SIPP系                     | 連続時間・固定軌道・実形状寄りへ広げやすい       | 実装が重くなりやすい            |
| 補助高速化                 | LNS, 学習支援                       | anytime改善、探索支援              | 単独主役より補助向き            |

この表のうち LaCAM* は、特に **「anytimeで、まず何か返せる候補」** として加えるのが筋です。LaCAM* は文献中でも eventually optimal を狙う方式として位置づけられており、最近の論文でも強いベースラインあるいは expert solver として使われています。 ([IJCAI][2])

あなたの要件に対する候補順位も少し変わります。
もし重みが **説明性・安全側の論理構造** に強くあるなら、依然として **CBS系** が第一候補です。
一方、**100ms級でまず返す・中断時にも解が欲しい・PoCで現実的な品質を見たい** をより強く置くなら、**LaCAM*** はかなり有力な比較対象になります。特に「ベンダーに候補アルゴリズムを提案してもらう」場では、CBS系だけだと保守的すぎ、WHCA*系だけだと最適性や説明性が弱すぎるので、**LaCAM* を第3の柱として入れる** のは合理的です。 ([IJCAI][2])

私なら、ベンダー比較用の候補セットを次の4本にします。

* **CBS / ICBS 系**: 説明性と制約処理の基準
* **ECBS / EECBS 系**: 速度と品質の現実解
* **LaCAM***: anytime・大規模寄り探索の有力候補
* **CCBS / SIPP 系**: 連続時間・固定軌道・実機寄り拡張の軸

この4本で比較すると、「論理的に説明しやすい系」「速い系」「anytime系」「連続時間系」が一通り揃います。 ([IJCAI][2])

要するに、**LaCAM*** は単なる追加候補ではなく、
あなたの要求のうち **anytime性** と **現実的計算時間内での解の質** を見る上で、かなり重要な比較対象です。
今後の整理では、LaCAM / LaCAM* を **「configuration-based / anytime MAPF」** という独立カテゴリで扱うのがよいです。 ([AAAI Digital Library][1])

必要なら次に、
**CBS系 / ECBS系 / LaCAM* / CCBS系 を、あなたの15要件に対して比較した選定表** を作ります。

[1]: https://ojs.aaai.org/index.php/AAAI/article/view/26377/26149?utm_source=chatgpt.com "Search-Based Algorithm for Quick Multi-Agent Pathfinding"
[2]: https://www.ijcai.org/proceedings/2023/0028.pdf?utm_source=chatgpt.com "Improving LaCAM for Scalable Eventually Optimal Multi- ..."



