# 2026 UPC

物理建模与英文论文项目，已选择 A 题：一级火箭助推器回收。Team Control Number：758。当前为论文框架阶段，尚无最终数值结论。

## 队内安排

[A 题解题路线与三人分工](docs/problem-a-team-plan.md)汇总了首条着陆轨迹的目标、三人交接内容和交稿节点。先确定主情景与着陆条件，再并行推进参数、轨迹和论文。

## 文件组织

- `figure/problem.png`：原始题图（含 A、B 两题）。
- `ref/UPC.pdf`：用户提供的去年银奖参考论文，仅参考组织结构。
- `docs/problems.md`：题意、已知量与建模入口。
- `paper/main.tex`：英文论文骨架，按参考论文的摘要、假设、符号、物理模型、计算、评价、参考文献和附录组织。
- `src/`：后续求解代码。
- `results/`：后续可复现计算结果。

## 论文编译
安装含 latexmk 的 TeX 发行版，在项目根目录运行：

```powershell
latexmk -pdf -outdir=paper/build paper/main.tex
```

当前只提交源文件；尚未验证 PDF 编译和排版。模板不代表赛事官方格式，提交前需核对当届要求。

## 当前平面刚体参考模型
参数：config/reference.json；说明：docs/planar-model.md。运行：node src/run-planar.mjs。包含轴向/横流、有动力制动及俯仰；全程可行轨迹仍待优化。
