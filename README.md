# Dynamic Model with NN Truck

三轴车辆动力学轨迹数据采集工程。基于 **TruckSim + Simulink** 联合仿真，在不同车型参数与工况下批量生成状态轨迹，用于后续机理模型对比与残差神经网络建模。

---

## 核心链路

```text
Baseline #2.cpar  →  trucksim_different_condition.slx  →  Truck_simulate_different_condition.m
        ↑                         ↑                              ↓
   TruckSim 车型包          Signal Editor 工况输入          data/.../*.mat 轨迹
                              (scenarios_input)
```

| 文件 | 作用 |
|---|---|
| `Baseline #2.cpar` | **采集时必须加载的 TruckSim 车型**（三轴 Baseline 2） |
| `trucksim_different_condition.slx` | Simulink–TruckSim 联合仿真模型，含多组 Signal Editor |
| `Truck_simulate_different_condition.m` | 批量切换场景、跑仿真、降采样并保存 `.mat` |
| `data/.../scenarios_input/` | 各工况的 Signal Editor 场景输入（`.mat`） |
| `data/trucksim/` | 采集得到的轨迹数据 |
| `Three_axle_model/` | 三轴机理模型与 TruckSim 数据对比脚本 |

> 采集前请在 TruckSim 中确认加载的是 **`Baseline #2.cpar`**，再与 `trucksim_different_condition.slx` 联仿。

---

## 快速开始

1. 打开 TruckSim，导入并激活 **`Baseline #2.cpar`**
2. 用 MATLAB 打开工程根目录，确认 `trucksim_different_condition.slx` 可正常联仿
3. 编辑 `Truck_simulate_different_condition.m`：
   - 选择 `signalEditorBlockPath`（对应转向模式 / 工况组）
   - 指定 `scenarioNames`（或依赖自动读取）
   - 修改 `save_dir`（输出目录与车型参数标签）
4. 运行脚本，等待进度条结束；结果写入 `data/trucksim/different_scenarios/...`

脚本默认流程：按场景循环 → `sim(model)` → 控制量与状态 **隔 10 点取 1**（约 0.01 s）→ 按场景名保存 `.mat`。

---

## 工况与输入

场景输入位于：

```text
data/trucksim/different_scenarios/scenarios_input/
```

常见工况包括：直行 / 蛇形、前轴转向、前中轴转向、反向转向、蟹行、横移、左右转、坦克掉头、侧翻蛇形等。  
在 `.m` 中通过切换 Signal Editor 块路径选择工况族，例如：

- `Signal Editor-opposite_steering`
- `Signal Editor-only_front_steering`
- `Signal Editor-front_middle_steering`
- `Signal Editor-same_steering_crab_motion`
- `Signal Editor-tank_turn`
- `Signal Editor-lateral_movement`

---

## 数据说明

### 目录约定

```text
data/trucksim/
├── different_scenarios/          # 多工况、多车型参数批量采集
│   ├── scenarios_input/          # Signal Editor 输入
│   ├── data_scenarios_for_* /    # 按工况命名的数据集
│   └── .../VehicleParams_*/      # 按车型参数标签分子目录
└── specific_control/             # 定工况对照数据（较早批次，状态为 33 维）
```

`VehicleParams_*` 目录名标记当前 TruckSim 参数组合（如惯量、滚动阻力、气动、簧上质量、轴距等档位）。

### 单条 `.mat` 字段

| 变量 | 含义 |
|---|---|
| `T_motor` | 六轮电机转矩，形状 `(N, 6)`，单位 Nm |
| `Delta_act` | 六轮转角，形状 `(N, 6)`，单位 rad |
| `Vehicle_state_trucksim_39d` | TruckSim 车辆状态轨迹，形状 `(N, 39)` |
| `vehicle_params` | 记录用整车参数（质量、轴距、轮径等） |
| `initial_conditions` | `vx0 / vy0 / wz0 / w0` |
| `sim_params` | 仿真时长、求解器、步长 |
| `specific_control_params` | 含当前 `ActiveScenario` 等 |

早期 `specific_control` 数据使用 `Vehicle_state_trucksim_33d`；当前主采集链路为 **39 维**状态。

轮序统一为：`FL, FR, ML, MR, RL, RR`。

---

## 机理模型（可选）

`Three_axle_model/` 提供线性自行车模型、Pacejka 三轴模型，以及与 TruckSim 轨迹的对比工具（`parse_data.py`、`trucksim_rule_compare.py` 等），便于验证规则模型与标定残差学习目标。

---

## 环境

- MATLAB / Simulink（含 Signal Editor）
- TruckSim（加载 `Baseline #2.cpar`）
- Python 3（可选，用于 `Three_axle_model` 解析与对比）

---

## 备注

- 旧版 CarSim / 定工况脚本与模型已归档至 `backup/`
- 大体积压缩包（如 `*_rollover_v2.zip`）默认不纳入版本库；请直接使用解压后的 `.mat` 目录
