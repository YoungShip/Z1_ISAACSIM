# Z1_ISAACSIM（Unitree Z1 + D435i）Isaac Sim Extension

这是一个 Isaac Sim 5.1 的 Extension 示例：
- 通过 **URDF Importer** 导入宇树 **Z1 机械臂（带夹爪）**
- 集成 **RMPflow** 做末端跟随目标
- 集成 **RealSense D435i** 结构（URDF + `d435.dae` 网格）
- 自动创建 `rgb_camera` / `depth_camera`（USD Camera prim），并做了朝向固定与可见性处理

> 说明：USD 里没有“深度相机”这种不同类型的 prim。`depth_camera` 仍然是 `Camera` prim，深度输出来自渲染管线的 depth AOV；此示例里主要通过 `ClippingRange` 体现深度量程。

---

## 环境要求
- **OS**：Ubuntu 22.04
- **Isaac Sim**：5.1.x
- **Python**：Isaac Sim 内置 Python（3.11）

---

## 仓库结构（最新）

```text
Z1_ISAACSIM/
├── README.md
├── assets/
└── exts/
    └── isaacsim.import_z1/
        ├── config/
        │   └── extension.toml
        ├── data/
        │   └── z1_description/
        │       ├── urdf/
        │       │   ├── z1_with_gripper.urdf
        │       │   └── z1_with_gripper_cam_final.urdf    # ✅ 带 D435i
        │       └── meshes/
        │           ├── visual/
        │           │   ├── d435.dae                      # ✅ D435i 网格
        │           │   ├── z1_Link00.dae ... z1_Link06.dae
        │           │   ├── z1_GripperMover.dae
        │           │   └── z1_GripperStator.dae
        │           └── collision/...
        └── isaacsim/
            └── import_z1/
                ├── __init__.py
                ├── common.py
                ├── import_z1.py                          # ✅ 主逻辑
                └── rmpflow/...
```

---

## 部署/启用（在 Isaac Sim 中加载源码扩展）

1. 启动 Isaac Sim。
2. 菜单：`Window` → `Extensions`。
3. 点击右上角 **Settings（齿轮）**。
4. 在 **Extension Search Paths** 中点击 `+`，添加本仓库的 `exts/` 目录：
   - 例如：`/home/<user>/Projects/Z1_ISAACSIM/exts`
5. 在 Extensions 搜索框输入 `z1`，启用扩展：`Unitree Z1 Import`（名称以 `extension.toml` 为准）。
6. 打开示例：`Isaac Examples` → `Import Robots` → `Z1 URDF`。

---

## 使用说明（示例 UI 按钮）

- **Load Robot**：导入 URDF（默认使用 `data/z1_description/urdf/z1_with_gripper_cam_final.urdf`）。
- **Configure Drives**：配置 6 轴 + 夹爪关节驱动参数。
- **Move to Pose**：演示驱动到一组姿态。
- **Control Gripper**：开合夹爪。
- **RMPflow Follow Target**：创建目标方块并启动物理步回调，末端跟随目标。
- **Toggle Debug Mode**：调试输出。
- **Reconfigure Camera**：删除并重建相机相关 prim。

---

## D435i 相机：资源、路径与行为

### 资源放置
- `exts/isaacsim.import_z1/data/z1_description/meshes/visual/d435.dae`

### URDF 引用（必须是相对 package 路径）
- `package://z1_description/meshes/visual/d435.dae`

### 相机 prim
- `.../camera_color_optical_frame/rgb_camera`
- `.../camera_depth_optical_frame/depth_camera`

默认行为：
- **固定相机朝向（仅旋转，不改平移）**：`(90, 0, -90)`（度）
- **可见性**：`rgb_camera`、`depth_camera` 设为不可见（`Visibility=invisible`）

调试：每 2 秒打印一次：
- `[CAM_ROT] rgb=(rx,ry,rz) | depth=(rx,ry,rz)`

---

## 代码入口
- 主逻辑：`exts/isaacsim.import_z1/isaacsim/import_z1/import_z1.py`
- Extension 配置：`exts/isaacsim.import_z1/config/extension.toml`
