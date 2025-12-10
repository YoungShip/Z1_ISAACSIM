# Unitree Z1 Isaac Sim Extension (Z1 机械臂导入与控制扩展)

这是一个 Isaac Sim 的扩展 (Extension) 示例，用于演示如何**从源码构建**一个扩展，将宇树 (Unitree) Z1 机械臂导入仿真环境，并实现基于 RMPflow 的运动控制。

## 💻 本机开发环境
本扩展是在以下环境中开发和测试的：
*   **操作系统**: Ubuntu 22.04 LTS
*   **ROS 版本**: ROS2 Humble
*   **Isaac Sim 版本**: 5.1.0
*   **Python 版本**: 3.11.14 (Isaac Sim 内置)
*   **CUDA 版本**: 12.2 (Driver 535.216.01)
*   **显卡**: NVIDIA GeForce RTX 4070 Laptop GPU

## 📂 项目结构与文件说明
本仓库展示了一个标准的 Isaac Sim Extension 目录结构。如果你想从零开始构建这个扩展，请参考以下结构：

```text
Z1_ISAACSIM/
└── exts/                               # 扩展存放的根目录
    └── isaacsim.import_z1/             # [核心] 扩展主目录 (Extension Root)
        ├── config/
        │   └── extension.toml          # [配置文件] 扩展的元数据（名称、依赖、模块入口）
        ├── data/                       # [资源目录] 存放机器人模型文件
        │   └── z1_description/
        │       ├── urdf/               # URDF 文件 (z1_with_gripper.urdf)
        │       └── meshes/             # 模型网格文件 (.dae, .STL)
        └── isaacsim/                   # [代码目录] Python 包根目录 (Namespace Package)
            └── import_z1/              # 实际的 Python 模块
                ├── __init__.py         # 模块初始化
                ├── import_z1.py        # [主逻辑] 扩展的核心代码（UI、加载逻辑、RMPflow集成）
                ├── common.py           # 通用工具函数
                └── rmpflow/            # [算法配置] RMPflow 算法所需的配置文件
                    ├── config.json
                    ├── z1_rmpflow_config.yaml
                    ├── z1_robot_description.yaml
                    └── lula_z1.urdf
```

## 🛠️ 如何从源码构建此扩展 (Implementation Guide)

如果你想利用本仓库的资源，手动在 Isaac Sim 中创建这个扩展，请按照以下步骤操作：

### 1. 资源放置 (Assets Placement)
*   **目标位置**: `exts/isaacsim.import_z1/data/z1_description/`
*   **操作**: 将 Z1 机械臂的 `urdf` 文件夹和 `meshes` 文件夹完整复制到上述目录下。
*   **注意**: 保持 `meshes` 和 `urdf` 的相对位置，以便 URDF 文件中的相对路径 (如有) 或 Isaac Sim 的解析器能正确找到网格。

### 2. 配置 Extension.toml
*   **文件位置**: `exts/isaacsim.import_z1/config/extension.toml`
*   **作用**: 告诉 Isaac Sim 这是一个扩展，以及如何加载它。
*   **关键内容**:
    ```toml
    [package]
    version = "1.0.0"
    title = "Unitree Z1 Import"
    # ... 其他元数据

    [dependencies]
    "isaacsim.asset.importer.urdf" = {}          # 依赖 URDF 导入器
    "isaacsim.robot_motion.motion_generation" = {} # 依赖 RMPflow 模块

    [[python.module]]
    name = "isaacsim.import_z1"                  # 指向 isaacsim/import_z1 目录
    ```

### 3. 编写主逻辑代码 (import_z1.py)
*   **文件位置**: `exts/isaacsim.import_z1/isaacsim/import_z1/import_z1.py`
*   **核心修改点**:
    *   **相对路径加载**: 为了让扩展在任何位置都能运行，**不要使用绝对路径**加载资源。使用 `__file__` 动态计算路径：
        ```python
        # 获取当前 py 文件所在目录的上三级目录，即扩展根目录
        ext_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        # 拼接得到 URDF 文件的绝对路径
        urdf_path = os.path.join(ext_path, "data/z1_description/urdf/z1_with_gripper.urdf")
        ```
    *   **加载 RMPflow**: 同样使用相对路径加载 `rmpflow` 文件夹下的配置文件。

### 4. 加载到 Isaac Sim (Loading Source)
完成上述文件创建后，按以下步骤加载源码：
1.  打开 Isaac Sim。
2.  顶部菜单 `Window` > `Extensions`。
3.  点击 **Settings (齿轮图标)**。
4.  在 **Extension Search Paths** 中，点击 `+` 号，选择本仓库的 `exts` 文件夹 (例如 `/home/user/projects/Z1_ISAACSIM/exts`)。
5.  在搜索栏搜 "Z1"，启用 **Unitree Z1 Import**。
6.  在 `Isaac Examples > Import Robots` 菜单下即可找到并运行。
