# 2026 UPC

物理建模与英文论文项目，已选择 A 题：一级火箭助推器回收。Team Control Number：758。P0 参考设计已冻结，P1 已验证耦合动力学，P2 已找到并复算全程触舰轨迹。P2 装载量对应参考设计和已搜索的策略。

## 队内安排

[A 题解题路线与三人分工](docs/problem-a-team-plan.md)汇总了首条着陆轨迹的目标、三人交接内容和交稿节点。先确定主情景与着陆条件，再并行推进参数、轨迹和论文。

## 文件组织

- `figure/problem.png`：原始题图（含 A、B 两题）。
- `ref/UPC.pdf`：用户提供的去年银奖参考论文，仅参考组织结构。
- `docs/problems.md`：题意、已知量与建模入口。
- `paper/main.tex`：英文论文骨架，按参考论文的摘要、假设、符号、物理模型、计算、评价、参考文献和附录组织。
- `src/`：P1 动力学、P2 策略搜索和优化代码。
- `results/p2-p0-v1/`：基于 P0 冻结参数和 P1 动力学的 P2 预算、控制与复算轨迹。
- [`docs/p2-landing-strategy.md`](docs/p2-landing-strategy.md)：P2 复现命令、优化结果和模型边界。

## 论文编译
使用含 XeLaTeX、fontspec 和 latexmk 的 TeX 发行版。在项目根目录运行以下命令。

```powershell
latexmk -xelatex -outdir=paper/build paper/main.tex
```

当前工作稿已编译并检查版面。已完成部分及剩余工作见各交接文档。

## 当前平面刚体参考模型
历史平面参数见 `config/reference.json`，模型见 `docs/planar-model.md`。运行 `node src/run-planar.mjs` 可重现 P1 演示。全程优化与复算见 P2 交付。

## P0 交接
P0历史参数入口为 config/reference.json；场景、预算及约束规范为 config/p0-contract.json。阅读 [P0交接说明](docs/p0-handoff.md) 后开展P2；旧 baseline.json 仅作历史校验。

## P1 交付
推导与内部验证已完成，见 [P1验证说明](docs/p1-verification.md)。运行 `node src/verify-p1.mjs` 重现检查；`python src/plot-p1.py` 生成三张论文配图（需matplotlib、numpy）。独立P1报告源文件为 paper/p1-report.tex；完整论文通过input复用正文。P1不包含全程轨迹优化或最终燃料答案。

## P3 本轮交付

[P3 收尾交接](docs/p3-closeout.md)与[正文修整记录](docs/p3-editorial-revision.md)说明当前主线：发动机与阻力假设怎样改变分离装载量，以及同一条名义轨迹的全程载荷。限定计算与正文整理已交付；全程结构与热安全仍未闭合，不能据此声称已证明安全回收。[结论边界](docs/p3-conclusion-boundaries.md)逐项区分假设、范围外扩展和核心未解事项。

- [发动机效率与推进剂需求](docs/p3-propulsion-sensitivity.md)：三档已知有效比冲的局部设计库存为 28.781、26.228、24.199 t，每档分别选择合适船位。
- [限定阻力对比](docs/p3-drag-sensitivity.md)：横流系数 1.6 的已接受局部设计为 27.130 t；系数 0.8 的限时搜索未得到通过候选，不作为燃料需求。
- [载荷计算记录](docs/p3-flight-safety.md)：正文已改用 26.228 t 名义轨迹，与燃料预算对应；结构和热安全尚未确定。
- [英文 P3 正文](paper/p3-sensitivity.tex)及[论文工作稿](output/pdf/758A-P3-progress.pdf)。

### 补充计算记录

以下材料保留为扩展记录，不作为名义燃料答案或最终回收方案的组成部分。正文仅保留一段有限反馈示例。

- [分离时重规划](docs/p3-control-revision.md)及[逐情景结果](results/p3/replanned/summary.json)：30 t 共同试算库存、固定船位、准确初态和恒定风已知。
- [风估计误差扫描](docs/p3-wind-estimation.md)：固定 30 t 计划下的输入误差影响。
- [持续反馈示例](docs/p3-tracking-control.md)：带调节余量的参考轨迹及五个小风误差点，不能推广为普遍鲁棒结论。

## 整合论文
英文审阅稿为 output/pdf/758A.pdf，整合入口 paper/main.tex。最后核查和提交材料状态见 docs/p4-final-check.md。运行 node src/p4-requirements.mjs 可复现控制与几何需求检查。技术结论以文中参考模型条件为限，AI完整对话披露尚需队伍补齐。

## 当前名义解已采用球形地球
当前参数为config/spherical.json，说明见docs/spherical-correction.md。球面重优化并补跑0.40 s网格后得到25.967 t总推进剂、284.120 km地表弧长船位。结构能力按用户研究范围排除。

## P5 修订（任务书 docs/p5-task-list.md，收尾说明与仍存问题见 docs/p5-closeout.md）
分离角、比冲、横流系数和动压上限都已在球面模型上重算，汇总见 results/spherical/summary.json。大气在84.852 km以上可选等温外推（upperAtmosphere: isothermal），30°情景因此不再被迫上升段制动。output/pdf/758A.pdf 已按最新源文件重新编译。本机可运行 `python src/build-paper.py`，使用 `.tools` 中的便携 TinyTeX 和 XeLaTeX。

