# Mini Control & Automation（迷你控制与自动化）

**从零开始、零依赖的 C 语言实现**，覆盖控制理论与工业自动化——从经典 PID 整定和状态空间方法，到智能控制、电机驱动、PLC 编程、DCS/SCADA 系统，以及机器人运动学/动力学。每个模块对应 MIT、Stanford 及其他顶尖大学的课程，将教科书公式和工业标准转化为可运行的 C 代码。

## 子模块总览

| 子模块 | 主题 | 参考课程 |
|--------|------|----------|
| [mini-classical-control](mini-classical-control/) | 传递函数、状态空间模型、拉普拉斯域、零极点分析、Routh-Hurwitz 判据、Nyquist/Bode 稳定性分析、根轨迹、PID 补偿器设计、直流电机/巡航/温度/位置伺服控制 | MIT 6.302, MIT 6.241J |
| [mini-dcs-scada](mini-dcs-scada/) | Modbus RTU/TCP、OPC UA、IEC 61158 现场总线、ISA-18.2 报警管理（消抖、洪峰抑制、搁置）、EEMUA 191、过程数据历史库（趋势分析、FDA 21 CFR Part 11）、工业通信协议 | MIT 6.02, Stanford EE379 |
| [mini-intelligent-control](mini-intelligent-control/) | 模糊逻辑控制、神经网络控制、自适应控制（MRAC、STR）、模型预测控制（MPC）、强化学习（Q-learning、SARSA）、滑模控制、迭代学习控制、遗传算法 | MIT 6.885, Stanford CS229 |
| [mini-modern-control](mini-modern-control/) | 状态空间模型（连续/离散/采样）、能控性与能观性、LQR、卡尔曼滤波器、极点配置、Lyapunov 稳定性、Ackermann 公式、Luenberger 观测器、分离原理 | MIT 6.241J, MIT 16.30, Stanford EE363 |
| [mini-motor-control](mini-motor-control/) | 直流电机/PMSM/感应/步进电机模型、Park/Clarke 变换、FOC 矢量控制、带抗积分饱和的 PID、级联电流/速度/位置环、MTPA、弱磁控制、BLDC 六步换相、霍尔传感器解码 | MIT 6.685, Berkeley EE117, ETH 227-0216 |
| [mini-pid-tuning](mini-pid-tuning/) | PID 结构（串联/并联/理想）、Ziegler-Nichols 与 Cohen-Coon 整定法、串级控制、前馈、增益调度、比值/选择控制、自适应 PID、模糊 PID、分数阶 PID、Bode/Nyquist 稳定裕度、自整定 | MIT 6.302, Stanford EE207 |
| [mini-plc-automation](mini-plc-automation/) | IEC 61131-3（梯形图 LD、FBD、ST、SFC、IL）、PLC 扫描周期（输入扫描→程序执行→输出刷新）、数字量 I/O、定时器/计数器、基于 DAG 的 FBD 求值、任务调度、工业自动化模式 | MIT 6.302 |
| [mini-robot-control](mini-robot-control/) | 正/逆运动学（DH 参数）、微分运动学（雅可比、伪逆）、Lagrangian 与 Newton-Euler 动力学、PD+重力补偿、计算力矩控制（CTC）、阻抗控制、自适应机械臂控制、轨迹生成 | Stanford CS223A, MIT 6.834 |

## 设计理念

- **零外部依赖** — 纯 C（C99/C11），仅使用 `libc` 和 `libm`
- **模块自包含** — 每个目录自带 `Makefile`、`include/`、`src/`、`examples/`、`demos/`、`tests/`
- **理论到代码的映射** — 每个模块直接对应大学级控制理论教材和工业标准（IEC、ISA）
- **实用演示程序** — 电机驱动仿真器、PLC 运行时引擎、SCADA 历史库、机器人轨迹规划器、PID 自整定器等

## 构建方式

每个模块相互独立。进入模块目录后运行：

```bash
cd mini-classical-control
make all    # 构建全部
make test   # 运行测试
```

需要 **GCC** 和 **GNU Make**。

## 项目结构

```
mini-control-automation/
├── mini-classical-control/     # 经典控制理论（拉普拉斯、Bode、Nyquist、根轨迹）
├── mini-dcs-scada/             # DCS/SCADA — 工业通信、报警管理、数据历史库
├── mini-intelligent-control/   # 智能控制 — 模糊、神经、自适应、MPC、强化学习、滑模
├── mini-modern-control/        # 现代控制 — 状态空间、LQR、卡尔曼、观测器
├── mini-motor-control/         # 电机控制 — FOC、PMSM、BLDC、级联控制环
├── mini-pid-tuning/            # PID 整定 — 高级结构、稳定性分析、自整定
├── mini-plc-automation/        # PLC 自动化 — IEC 61131-3、梯形图、FBD、ST、扫描周期
└── mini-robot-control/         # 机器人控制 — 运动学、动力学、CTC、阻抗控制
```

## 许可证

MIT
