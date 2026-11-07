# P5 任务书：论文剩余问题与缺口

依据 2026-11-07 对 `0dbe680` 版本论文的完成度与逻辑检查整理。每项写明问题、修改方式和验收标准。状态在完成后更新。

## A. 需要计算的项目

### T1 30° 情景受大气表上限约束（逻辑问题）
- 问题：`atmosphere.mjs` 在几何高度约 86 km 以上直接抛错，优化器把它当作硬约束。30° 情景因此被迫在分离时烧掉约 28 t 压低顶点，67 t 实际反映的是程序覆盖范围而非物理需要。
- 修改：在 84.852 km 以上用等温外推（186.87 K，按静力平衡指数衰减）延伸大气，JS 与 CasADi 两端一致；在球面模型下重算 30°（并补算 0°），得到滑行优先方案的装载量。
- 验收：旧 15° 解不受影响（全程低于 84.852 km）；新 30°/0° 候选在 0.005 与 0.0025 s 球面复算中通过全部触舰判据；正文表格与文字更新。

### T2 球面模型下的参数敏感性（完成度缺口）
- 问题：效率与横流比较只有平面结果，最终答案 26.038 t 没有对应的球面敏感性。
- 修改：以球面名义解热启动，分别重算 Isp 300/360 s 与 C_n 0.8/1.6，各自选择船位。
- 验收：通过两档复算的候选列入正文表格；未通过的如实标注。

### T3 动压约束的代价（"安全下降"论证缺口）
- 问题：名义轨迹峰值动压 293 kPa、比力 7.8 g，论文把结构排除在范围外，但题目要求"安全下降"。正文称"降低载荷需要提前制动"却无计算支持。
- 修改：在优化中加入动压上限约束（在再入减速点火情形下），计算若干上限对应的最小装载量，给出"动压上限—燃料"关系。
- 验收：至少一个带动压上限的候选通过球面复算；正文新增小节说明燃料代价，并把限值写成可选设计要求而非已验证结构能力。若求解失败，如实写明。

## B. 文字修改

### T4 球面修正后的遗留表述
- `main.tex` 假设一节仍称"基准平动模型假设平面运动"；`p1-verification.tex` 末段列"局部平面近似"并称全程可行性留给下一阶段；"优缺点"一节未提球面接触与地表弧长。
- 验收：全文对"当前参考模型 = 非自转球形地球；平面模型 = 初始化与比较"的口径一致。

### T5 载荷一节自相矛盾
- `p3-safety.tex` 称载荷与燃料用同一条 26.228 t 轨迹，但最终燃料答案已是 26.038 t 球面解。
- 修改：表格改为球面名义解数据，平面值作对照。

### T6 15° 参考分离角缺少理由
- 修改：在问题重述/假设中说明 15° 为代表性上升段分离角的选择依据，并把分离角对装载量的影响（含 T1 结果）作为主要不确定性写入摘要与结论。

### T7 符号表与术语
- 补全正文使用的符号（s, θ, ω, δ, u_T, L, D, R_b, R_E, C_a, C_n, Δl, I_d, A_end 等），删去未使用的 C_D、A；统一 "specific force"。

### T8 内部阶段代号与硬编码引用
- 正文中 "P0 criteria""P1 replay""P2 single-ignition family" 等改为读者可理解的名称；"Section 6" 改为 `\ref`。

## C. 收尾

### T9 编译与逐页检查
- 安装可用的 TeX 引擎，编译 `paper/main.tex`，检查未定义引用、溢出与分页，更新 `output/pdf/758A.pdf`。

### T10 更新记录文档
- 更新 `ref/todo/README.md`、`README.md`、相关 docs，记录本轮修改与复现命令。

### T11 AI 完整对话记录（队伍负责）
- 竞赛规则要求附上原始提问与完整输出。只能由队员导出各自的完整对话后补入附录，本轮无法代办。

## 状态
| 编号 | 状态 | 结果 |
|---|---|---|
| T1 | 完成 | 大气在 84.852 km 以上可选等温外推（`upperAtmosphere: "isothermal"`，默认关闭，P1 验证和旧结果不变）。球面模型下 0° 为 24.316 t（船位 199.26 km），30° 为 32.460 t（船位 348.98 km，顶点 107.7 km，无需上升段制动），原平面 30° 为 67.00 t。正文见 `paper/separation-angle.tex`。 |
| T2 | 完成 | 球面敏感性：Isp 300/360 s 为 28.483/23.978 t；C_n 0.8/1.0/1.6 为 25.940/25.956/25.495 t。原平面模型未求出的 C_n=0.8 改从 C_n=1.0 的解热启动后求出。全部通过 0.005 与 0.0025 s 复算。 |
| T3 | 完成（上界估计） | 优化器加入可选动压路径约束（`max_q`）。200 kPa 为 37.302 t，150 kPa 为 50.169 t，均为二次点火，通过两档复算且连续峰值不超上限。两例在最后一轮仍在下降，按用户同意停止，正文写作上界估计。 |
| T4 | 完成 | 假设一节、验证一节末段和"优缺点"一节已统一为"参考 = 非自转球形地球，平面 = 初始化与比较"。 |
| T5 | 完成 | 载荷表改为球面名义解数据（292.9 kPa、7.84 g0、9.98 MN），平面值放在表注里。 |
| T6 | 完成 | 说明分离方向由上升任务决定，不是回收方案的优化变量；15° 为假设的中等爬升角，并写入摘要与结论作为最大不确定性。 |
| T7 | 完成 | 符号表补全，删去未使用的 C_D、A，统一使用 "specific force"。 |
| T8 | 完成 | 正文内部代号已去除，章节引用改为 `\ref`；静态检查无未定义引用、重复标签或未定义文献。 |
| T9 | 完成 | 用 `python src/build-paper.py` 编译（经 Python 调用 `.tools/TinyTeX` 的 pdflatex，直接从 PowerShell 启动会被本机安全软件拦截），已覆盖 `output/pdf/758A.pdf`，共 30 页。无未定义引用、无溢出行，字体全部嵌入，每页页眉含队号。文字与数字已核对，页面图像未做目视检查。 |
| T10 | 完成 | 已更新 `README.md`、`ref/todo/README.md`、`docs/spherical-correction.md`。 |
| T11 | 已起草，待队伍确认 | `paper/ai-report.tex` 收录本机的 Codex 会话（41 轮）和 Kiro 会话（8 轮）：提问为英文译文，回复为摘要，姓名、路径、账号已替换为占位。文末留有占位，供补入其他队员的会话。原始日志在 `~/.codex/sessions/2026/11/07/` 和 DSH 会话记录中。 |

另外，名义解在 0.40 s 网格上补跑三轮（26.020、25.967、26.011 t），选最低的 25.967 t，船位 284.120 km；`selected.json`、图和 `spherical-results.tex` 已据此重新生成。

## 复现命令
在仓库根目录运行。CasADi 依赖需在 `PYTHONPATH` 中，本机放在 `paper/build/feedback-deps`。

```powershell
node src/spherical-angle-search.mjs 0; node src/spherical-angle-search.mjs 30
python src/spherical-angle-optimize.py 0; python src/spherical-angle-optimize.py 30
python src/spherical-sensitivity.py isp-300 ispS 300 --steps 0.55 0.4 0.4 0.4 0.4
python src/spherical-sensitivity.py isp-360 ispS 360 --steps 0.55 0.4 0.4 0.4 0.4
python src/spherical-sensitivity.py cn-1.0 crossflowCd 1.0 --wall 600
python src/spherical-sensitivity.py cn-0.8 crossflowCd 0.8 --wall 600 --seed results/spherical/sensitivity/cn-1.0.json
python src/spherical-sensitivity.py cn-1.6 crossflowCd 1.6 --steps 0.55 0.4 0.4 0.4 --wall 600
python src/spherical-sensitivity.py cn-1.2 crossflowCd 1.2 --steps 0.4 0.4 0.4 --wall 600
node src/spherical-q-search.mjs 200; node src/spherical-q-search.mjs 150
python src/spherical-q-optimize.py 200; python src/spherical-q-optimize.py 150
python src/spherical-report.py; python src/spherical-summary.py; python src/spherical-tables.py
```

优化脚本发现已有结果文件时会从最优已接受解继续，重复运行会追加信赖域轮次。
