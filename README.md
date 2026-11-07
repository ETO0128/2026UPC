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
安装含 latexmk 的 TeX 发行版，在项目根目录运行：

```powershell
latexmk -pdf -outdir=paper/build paper/main.tex
```

当前工作稿已编译并检查版面。已完成部分及剩余工作见各交接文档。

## 当前平面刚体参考模型
参数见 `config/reference.json`，模型见 `docs/planar-model.md`。运行 `node src/run-planar.mjs` 可重现 P1 演示。全程优化与复算见 P2 交付。

## P0 交接
当前参数入口为 config/reference.json；场景、预算及约束规范为 config/p0-contract.json。阅读 [P0交接说明](docs/p0-handoff.md) 后开展P2；旧 baseline.json 仅作历史校验。

## P1 交付
推导与内部验证已完成，见 [P1验证说明](docs/p1-verification.md)。运行 `node src/verify-p1.mjs` 重现检查；`python src/plot-p1.py` 生成三张论文配图（需matplotlib、numpy）。独立P1报告源文件为 paper/p1-report.tex；完整论文通过input复用正文。P1不包含全程轨迹优化或最终燃料答案。

## P3 本轮交付

[控制修正与局限](docs/p3-control-revision.md)说明固定原船位、统一 30 t 试算库存的分离时重规划。主情景和十个单因素扰动均满足 P0 触舰条件，前提为初态和恒定风准确已知。持续扰动、联合误差、最低共同库存及结构和热条件仍待完善。

- [完整轨迹复算结果](results/p3/replanned/summary.json)及同目录中的逐情景控制节点。
- [英文 P3 正文](paper/p3-sensitivity.tex)。
- [本轮 PDF 工作稿](output/pdf/758A-P3-progress.pdf)。
