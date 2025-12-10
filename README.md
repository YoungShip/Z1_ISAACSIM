# Unitree Z1 Isaac Sim 扩展

这是一个 Isaac Sim 扩展，用于导入宇树 (Unitree) Z1 机械臂（带夹爪），并提供基于 RMPflow 的目标跟随控制功能。

## 功能特性
- **URDF 导入**: 加载带夹爪的 Z1 机械臂模型。
- **关节控制**: 提供 UI 按钮来设置关节角度和控制夹爪开合。
- **RMPflow 控制**: 集成 Isaac Sim 的 RMPflow 算法，实现具有自动避障功能的末端目标跟随。

## 安装步骤
1. 克隆此仓库到本地：
   ```bash
   git clone https://github.com/NicoleKJ9721/Z1_ISAACSIM.git
   ```
2. 打开 Isaac Sim，在顶部菜单选择 `Window` > `Extensions`。
3. 点击扩展窗口右上角的 **齿轮图标 (Settings)**。
4. 在 **Extension Search Paths**（扩展搜索路径）中，点击绿色加号，添加本仓库下的 `exts` 文件夹路径（例如 `/你的路径/Z1_ISAACSIM/exts`）。
5. 在扩展搜索栏中输入 "Z1"，找到 `Unitree Z1 Import` 扩展并启用开关。

## 使用方法
1. 启用扩展后，通过菜单打开：`Isaac Examples` > `Import Robots` > `Z1 URDF`。
2. 点击 **Load Robot** 按钮加载 Z1 机械臂。
3. 点击 **RMPflow Follow Target** 按钮启用跟随模式。场景中会出现一个目标方块。
4. 拖动目标方块，机械臂末端将自动跟随并避开障碍物。

## 文件结构说明
本扩展包含运行所需的所有资源（模型、配置文件），无需依赖外部 ROS 环境或绝对路径。
