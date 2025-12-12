# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ============================================================================
# 文件说明：Isaac Sim 宇树 Z1 机械臂 URDF 导入示例
# 功能：演示如何通过 URDF 文件导入宇树 Z1 机械臂（带夹爪），并配置关节驱动器
# ============================================================================

# 导入异步编程模块，用于处理异步操作（如场景加载）
import asyncio
# 导入数学模块，用于角度转换等数学运算
import math
# 导入弱引用模块（虽然导入但未使用）
import weakref
import sys

# 导入 Omni 核心模块
import omni
# 导入 Omni UI 模块，用于创建用户界面
import omni.ui as ui
# 物理接口，用于订阅物理步回调
import omni.physx as physx
# 从 Isaac Sim 示例浏览器模块导入获取浏览器实例的函数
from isaacsim.examples.browser import get_instance as get_browser_instance
# 从 Isaac Sim GUI 组件工具模块导入按钮构建器、样式获取器和 UI 头部设置函数
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers
# 从 Omni Kit 视口工具模块导入视口相机状态类，用于控制相机位置
from omni.kit.viewport.utility.camera_state import ViewportCameraState
# 机器人运动生成与 RMPflow
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
# 从 USD (Universal Scene Description) 模块导入：
# - Gf: 几何和数学基础类（向量、矩阵等）
# - PhysxSchema: PhysX 物理引擎的 USD schema
# - Sdf: Scene Description Foundation，USD 的基础数据结构
# - UsdLux: USD 光照相关类
# - UsdPhysics: USD 物理相关类
# - UsdGeom: 用于创建/读取几何体（目标方块）
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics, UsdGeom

# 导入 NumPy，用于数值计算
import numpy as np

# 从当前包的 common 模块导入设置驱动器参数的辅助函数
from .common import set_drive_parameters

# 扩展名称常量
EXTENSION_NAME = "Import Z1"


class Extension(omni.ext.IExt):
    """
    Isaac Sim 扩展类，用于导入和配置宇树 Z1 机械臂（带夹爪）
    
    这个类继承自 omni.ext.IExt，是 Isaac Sim 扩展的标准基类。
    它负责：
    1. 注册示例到 Isaac Sim 的示例浏览器
    2. 创建用户界面
    3. 处理 URDF 文件的导入
    4. 配置机械臂的关节驱动器（包括 6 个主要关节和 1 个夹爪关节）
    """
    
    def on_startup(self, ext_id: str):
        """
        扩展启动时调用的初始化方法
        
        Args:
            ext_id: 扩展的唯一标识符字符串
        """
        # 获取扩展管理器，用于管理扩展的配置和路径
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        # 保存扩展 ID，用于后续操作
        self._ext_id = ext_id
        # 获取扩展的安装路径，用于访问扩展内的资源文件（如 URDF 文件）
        self._extension_path = ext_manager.get_extension_path(ext_id)
        # 初始化窗口变量为 None（当前未使用窗口）
        self._window = None

        # -------- RMPflow 相关成员变量 --------
        # Z1 articulation 封装
        self._z1_articulation: SingleArticulation | None = None
        # RMPflow 对象与控制器
        self._rmp_flow = None
        self._rmp_controller: mg.MotionPolicyController | None = None
        # 物理步订阅，用于在每个 physics step 中调用 RMPflow
        self._rmp_physx_subscription = None
        # 跟随目标的 USD 路径
        self._rmp_target_prim_path = "/World/Z1_RMP_Target"
        # 夹爪目标位置（用于在 RMPflow 控制时锁定夹爪）
        self._gripper_target_position = 0.0  # 默认关闭位置
        # 调试模式：设置为 True 可以在控制台看到夹爪锁定信息
        self._debug_gripper_lock = False  # 改为 True 启用详细调试输出

        # -------- 相机调试：周期性打印姿态 --------
        self._rgb_camera_prim_path: str | None = None
        self._depth_camera_prim_path: str | None = None
        self._camera_pose_print_task: asyncio.Task | None = None

        # 设置示例名称，显示在 Isaac Sim 的示例浏览器中
        self.example_name = "Z1 URDF"
        # 设置示例分类，用于在浏览器中分组显示
        self.category = "Import Robots"

        # 注册示例到 Isaac Sim 的示例浏览器
        # 这样用户就可以在 Isaac Sim 的示例浏览器中找到并运行这个示例
        get_browser_instance().register_example(
            name=self.example_name,  # 示例显示名称
            execute_entrypoint=self._build_window,  # 执行入口点函数（创建窗口）
            ui_hook=self._build_ui,  # UI 钩子函数（构建用户界面）
            category=self.category,  # 示例分类
        )

    def _build_window(self):
        """
        构建窗口方法（当前未使用，保留为空实现）
        
        这个方法原本用于创建一个独立的窗口，但当前实现中窗口功能被注释掉了。
        如果需要创建独立窗口，可以取消注释下面的代码。
        """
        # 创建窗口的代码（已注释）
        # self._window = omni.ui.Window(
        #     EXTENSION_NAME,  # 窗口标题
        #     width=0,  # 宽度为 0 表示自动调整
        #     height=0,  # 高度为 0 表示自动调整
        #     visible=False,  # 初始不可见
        #     dockPreference=ui.DockPreference.LEFT_BOTTOM  # 停靠位置：左下角
        # )
        pass

    def _build_ui(self):
        """
        构建用户界面方法
        
        创建示例的用户界面，包括：
        1. 标题和文档链接
        2. 四个功能按钮：加载机器人、配置驱动器、移动到姿态、控制夹爪
        """
        # 使用垂直堆栈布局（VStack）来组织 UI 元素
        # spacing=5: 元素之间的间距为 5 像素
        # height=0: 高度为 0 表示自动调整高度
        with ui.VStack(spacing=5, height=0):
            # 设置 UI 标题和说明信息
            title = "Import a Z1 Robot via URDF"
            # 文档链接，指向 Isaac Sim 官方文档中关于 URDF 导入器的说明
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html"
            # 概述文本，说明这个示例的功能
            overview = "This Example shows you import a Unitree Z1 robot arm with gripper via URDF.\n\nPress the 'Open in IDE' button to view the source code."

            # 设置 UI 头部（标题、文档链接、概述等）
            setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

            # 创建一个可折叠的框架（CollapsableFrame），用于包含命令面板
            frame = ui.CollapsableFrame(
                title="Command Panel",  # 框架标题：命令面板
                height=0,  # 高度自动调整
                collapsed=False,  # 初始状态：展开（不折叠）
                style=get_style(),  # 使用默认样式
                style_type_name_override="CollapsableFrame",  # 样式类型名称覆盖
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,  # 水平滚动条：需要时显示
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,  # 垂直滚动条：始终显示
            )
        # 在框架内创建 UI 元素
            with frame:
                # 在框架内再创建一个垂直堆栈，用于排列按钮
                with ui.VStack(style=get_style(), spacing=5):
                    # 第一个按钮：加载机器人
                    dict = {
                        "label": "Load Robot",  # 按钮标签
                        "type": "button",  # 按钮类型
                        "text": "Load",  # 按钮显示的文本
                        "tooltip": "Load a Z1 Robot with Gripper into the Scene",  # 鼠标悬停提示
                        "on_clicked_fn": self._on_load_robot,  # 点击事件处理函数
                    }
                    btn_builder(**dict)  # 使用按钮构建器创建按钮，**dict 解包字典作为关键字参数

                    # 第二个按钮：配置驱动器
                    dict = {
                        "label": "Configure Drives",  # 按钮标签
                        "type": "button",  # 按钮类型
                        "text": "Configure",  # 按钮显示的文本
                        "tooltip": "Configure Joint Drives",  # 鼠标悬停提示：配置关节驱动器
                        "on_clicked_fn": self._on_config_robot,  # 点击事件处理函数
                    }
                    btn_builder(**dict)

                    # 第三个按钮：移动到指定姿态
                    dict = {
                        "label": "Move to Pose",  # 按钮标签
                        "type": "button",  # 按钮类型
                        "text": "Move",  # 按钮显示的文本
                        "tooltip": "Drive the Robot to a specific pose",  # 鼠标悬停提示：驱动机器人到特定姿态
                        "on_clicked_fn": self._on_config_drives,  # 点击事件处理函数
                    }
                    btn_builder(**dict)

                    # 第四个按钮：控制夹爪
                    dict = {
                        "label": "Control Gripper",  # 按钮标签
                        "type": "button",  # 按钮类型
                        "text": "Open/Close Gripper",  # 按钮显示的文本
                        "tooltip": "Open or close the gripper",  # 鼠标悬停提示：打开或关闭夹爪
                        "on_clicked_fn": self._on_control_gripper,  # 点击事件处理函数
                    }
                    btn_builder(**dict)

                    # 第五个按钮：RMPflow 目标跟随
                    dict = {
                        "label": "RMPflow Follow Target",
                        "type": "button",
                        "text": "RMPflow Follow",
                        "tooltip": "Use RMPflow to make Z1 end-effector follow a target with collision avoidance",
                        "on_clicked_fn": self._on_rmpflow_follow_button,
                    }
                    btn_builder(**dict)
                    
                    # 第六个按钮：切换调试模式
                    dict = {
                        "label": "Toggle Debug Mode",
                        "type": "button",
                        "text": "Debug: OFF",
                        "tooltip": "Toggle gripper lock debug output in console",
                        "on_clicked_fn": self._on_toggle_debug,
                    }
                    btn_builder(**dict)
                    
                    # 第七个按钮：重新配置 D435i 相机（通常不需要，相机会自动配置）
                    dict = {
                        "label": "Reconfigure Camera",
                        "type": "button",
                        "text": "Reconfigure Camera",
                        "tooltip": "Manually reconfigure D435i camera (usually not needed, auto-configured on load)",
                        "on_clicked_fn": self._on_reconfigure_camera,
                    }
                    btn_builder(**dict)

    def on_shutdown(self):
        """
        扩展关闭时调用的清理方法
        
        当扩展被卸载时，这个方法会被调用，用于清理资源：
        1. 从示例浏览器中注销示例
        2. 清理窗口引用
        """
        # 从示例浏览器中注销这个示例
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        # 将窗口引用设置为 None，释放资源
        self._window = None

        # 取消物理步订阅，避免悬挂回调
        self._rmp_physx_subscription = None
        self._rmp_controller = None
        self._z1_articulation = None

        # 停止相机姿态打印任务
        if self._camera_pose_print_task is not None:
            try:
                self._camera_pose_print_task.cancel()
            except Exception:
                pass
        self._camera_pose_print_task = None

    # ----------------------------------------------------------------------
    # 相机调试：每 2 秒打印一次 rgb/depth 相机姿态（同一行）
    # ----------------------------------------------------------------------
    def _format_camera_rotation_deg(self, prim) -> str:
        """从 prim 的 xformOps 中提取旋转（优先 RotateXYZ），返回 'rx,ry,rz'（度）。"""
        try:
            xform = UsdGeom.Xformable(prim)
            ops = xform.GetOrderedXformOps()
        except Exception:
            return "N/A"

        rot_xyz = None
        orient_q = None
        for op in ops:
            t = op.GetOpType()
            if t == UsdGeom.XformOp.TypeRotateXYZ:
                rot_xyz = op.Get()
            elif t == UsdGeom.XformOp.TypeOrient:
                orient_q = op.Get()

        if rot_xyz is not None:
            return f"{float(rot_xyz[0]):.2f},{float(rot_xyz[1]):.2f},{float(rot_xyz[2]):.2f}"

        if orient_q is not None:
            # quat: w, (x,y,z)
            w = float(orient_q.GetReal())
            x, y, z = orient_q.GetImaginary()
            x = float(x)
            y = float(y)
            z = float(z)

            # 四元数 -> 欧拉角（XYZ），输出度
            t0 = 2.0 * (w * x + y * z)
            t1 = 1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)

            t2 = 2.0 * (w * y - z * x)
            t2 = 1.0 if t2 > 1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)

            t3 = 2.0 * (w * z + x * y)
            t4 = 1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            return f"{math.degrees(roll_x):.2f},{math.degrees(pitch_y):.2f},{math.degrees(yaw_z):.2f}"

        return "0.00,0.00,0.00"

    async def _camera_pose_printer_loop(self):
        """后台循环：每 2 秒打印一次 rgb/depth 相机的旋转角度（同一行）。"""
        while True:
            try:
                stage = omni.usd.get_context().get_stage()
                rgb_path = self._rgb_camera_prim_path
                depth_path = self._depth_camera_prim_path

                if not rgb_path or not depth_path:
                    await asyncio.sleep(2.0)
                    continue

                rgb_prim = stage.GetPrimAtPath(rgb_path)
                depth_prim = stage.GetPrimAtPath(depth_path)

                if not rgb_prim.IsValid() or not depth_prim.IsValid():
                    await asyncio.sleep(2.0)
                    continue

                rgb_rot = self._format_camera_rotation_deg(rgb_prim)
                depth_rot = self._format_camera_rotation_deg(depth_prim)

                msg = f"[CAM_ROT] rgb=({rgb_rot}) | depth=({depth_rot})"
                # 同一行刷新输出（Kit 控制台通常会显示为一行不断更新）
                sys.stdout.write(msg + "\r")
                sys.stdout.flush()

            except asyncio.CancelledError:
                sys.stdout.write("\n")
                sys.stdout.flush()
                return
            except Exception as e:
                print(f"\n⚠️ 相机姿态打印异常: {e}")

            await asyncio.sleep(2.0)

    def _menu_callback(self):
        """
        菜单回调方法（当前未使用）
        
        如果通过菜单调用扩展，这个方法会切换窗口的可见性。
        """
        # 如果窗口不存在，先构建 UI
        if self._window is None:
            self._build_ui()
        # 切换窗口的可见性（显示/隐藏）
        self._window.visible = not self._window.visible

    def _on_load_robot(self):
        """
        加载机器人按钮的点击事件处理函数
        
        这个方法启动异步加载流程：
        1. 首先创建一个新的 USD 场景（stage）
        2. 然后在新场景中加载 Z1 机器人（带夹爪）
        """
        # 创建一个异步任务：获取新的 USD 场景
        # omni.usd.get_context().new_stage_async() 返回一个协程，用于异步创建新场景
        load_stage = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        # 创建另一个异步任务：加载机器人
        # 传入 load_stage 任务，等待场景创建完成后再加载机器人
        asyncio.ensure_future(self._load_robot(load_stage))

    async def _load_robot(self, task):
        """
        异步加载机器人方法
        
        这是实际执行机器人加载的异步方法。它：
        1. 等待场景创建完成
        2. 创建 URDF 导入配置
        3. 导入 URDF 文件
        4. 设置相机位置
        5. 创建物理场景和光照
        
        Args:
            task: 场景创建任务的 Future 对象
        """
        # 等待场景创建任务完成
        # asyncio.wait() 返回两个集合：已完成的任务和待完成的任务
        done, pending = await asyncio.wait({task})
        # 检查任务是否已完成
        if task in done:
            # 执行命令创建 URDF 导入配置
            # URDFCreateImportConfig 命令返回一个状态和导入配置对象
            status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            
            # 配置导入选项：
            # merge_fixed_joints: 是否合并固定关节
            # True = 合并固定关节，大幅减少相机 frame 节点数量（推荐）
            # False = 保留所有固定关节，场景树会包含所有相机坐标系 frame（用于 ROS TF）
            import_config.merge_fixed_joints = True  # 🔥 简化场景树，避免过多相机节点
            
            # fix_base: 是否固定基座（True = 固定基座，机器人不会掉落）
            import_config.fix_base = True
            
            # make_default_prim: 是否将导入的机器人设置为默认 prim（根节点）
            import_config.make_default_prim = True
            
            # create_physics_scene: 是否创建物理场景（True = 自动创建物理场景）
            import_config.create_physics_scene = True
            
            # import_inertia_tensor: 导入惯性张量（重要！）
            import_config.import_inertia_tensor = True
            
            # 🔥 关键配置：设置关节驱动器参数，让关节能够正常工作
            # 注意：不同版本的 Isaac Sim 可能有不同的配置方式
            # 这里使用通用的属性设置方式
            
            # 尝试设置默认驱动类型为位置控制（如果属性存在）
            try:
                # 方法 1：尝试使用字符串形式
                if hasattr(import_config, 'default_drive_type'):
                    import_config.default_drive_type = "position"
            except:
                pass
            
            # 设置驱动刚度（stiffness）：控制关节回到目标位置的强度
            if hasattr(import_config, 'default_drive_strength'):
                import_config.default_drive_strength = 1e7  # 位置控制刚度
            
            # 设置位置驱动阻尼（damping）：控制关节运动的阻力，减少震荡
            if hasattr(import_config, 'default_position_drive_damping'):
                import_config.default_position_drive_damping = 1e5
            
            # 单位缩放（URDF 通常是米制）
            if hasattr(import_config, 'distance_scale'):
                import_config.distance_scale = 1.0
            
            # 执行 URDF 解析和导入命令
            # URDFParseAndImportFile 命令会解析 URDF 文件并将其导入到 USD 场景中
            # 注意：URDF 文件路径需要是绝对路径
            urdf_path = "/home/kj/Desktop/Projects/Z1_isaacsim/unitree_ros/robots/z1_description/xacro/z1_with_gripper_cam_final.urdf"
            print(f"正在导入 Z1 机械臂（带相机）: {urdf_path}")
            omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=urdf_path,  # Z1 机械臂（带夹爪和 D435i 相机）的 URDF 文件路径
                import_config=import_config,  # 导入配置对象
            )
            print("✅ Z1 机械臂（带 D435i 相机）导入成功！")

            # 设置视口相机的位置和朝向
            # 获取默认透视相机的状态对象
            camera_state = ViewportCameraState("/OmniverseKit_Persp")
            # 设置相机在世界坐标系中的位置：稍微抬高一些以便看到地板
            # (x=2.0, y=-2.0, z=1.2) - 从机器人的右后上方观察
            camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 1.2), True)
            # 设置相机朝向的目标点：机器人基座位置，稍微抬高以看到整体
            camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.3), True)

            # 获取当前的 USD 场景（stage）
            stage = omni.usd.get_context().get_stage()
            
            # 创建物理场景（Physics Scene）
            # UsdPhysics.Scene.Define() 在指定路径定义物理场景
            scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
            # 设置重力方向：向下（Z 轴负方向）
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            # 设置重力大小：9.81 m/s²（地球标准重力加速度）
            scene.CreateGravityMagnitudeAttr().Set(9.81)

            # 创建远距离光源（平行光，类似太阳光）
            # UsdLux.DistantLight.Define() 在指定路径定义远距离光源
            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            # 设置光源强度：500（数值越大，光线越亮）
            distantLight.CreateIntensityAttr(500)
            # set light angle: 设置光源的方向向量
            distantLight.CreateAngleAttr().Set(-150)  # 光源角度（影响阴影柔和度）
            
            # 🌍 添加 Isaac Sim 默认环境作为地板
            # 这是一个带有网格地面和天空盒的标准环境
            print("🌍 正在加载地板环境...")
            environment_url = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Grid/default_environment.usd"
            try:
                # 创建一个 Xform 作为环境的父节点
                environment_xform = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Environment"))
                
                # 使用 Payload 引用外部 USD 文件（更高效，支持延迟加载）
                environment_prim = environment_xform.GetPrim()
                environment_prim.GetPayloads().AddPayload(environment_url)
                
                print(f"✅ 地板环境加载成功: {environment_url}")
            except Exception as e:
                print(f"⚠️ 地板环境加载失败: {e}")
                print("   机器人仍然可以正常使用，只是没有地板")
                # 如果加载失败，创建一个简单的地面平面作为备用
                try:
                    print("   正在创建备用地面...")
                    ground = UsdGeom.Plane.Define(stage, Sdf.Path("/World/GroundPlane"))
                    ground.CreateAxisAttr("Z")  # Z 轴朝上
                    ground.CreateExtentAttr([(-10, -10), (10, 10)])  # 20m x 20m 的地面
                    
                    # 添加碰撞属性（UsdPhysics 已在文件顶部导入）
                    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
                    print("   ✅ 备用地面创建成功")
                except Exception as ex:
                    print(f"   ⚠️ 备用地面创建也失败: {ex}")
                    pass
            
            # 🎥 自动配置 D435i 相机（在机器人加载后）
            # 延迟一帧，确保 URDF 完全加载
            await asyncio.sleep(0.1)
            self._setup_d435i_camera()

    def _on_config_robot(self):
        """
        配置机器人按钮的点击事件处理函数
        
        这个方法配置 Z1 机器人的物理和关节驱动器参数：
        1. 设置 PhysX 求解器的迭代次数（影响物理模拟精度和性能）
        2. 获取所有 6 个主要关节和 1 个夹爪关节的驱动器 API
        3. 为每个关节配置位置控制模式、刚度、阻尼和最大力
        
        Z1 的 6 个主要关节：
        - joint1: 基座旋转关节（Z 轴）
        - joint2: 肩部抬升关节（Y 轴）
        - joint3: 肘关节（Y 轴）
        - joint4: 腕部关节 1（Y 轴）
        - joint5: 腕部关节 2（Z 轴）
        - joint6: 腕部关节 3（X 轴）
        
        夹爪关节：
        - jointGripper: 夹爪开合关节（Y 轴）
        """
        # 获取当前的 USD 场景
        stage = omni.usd.get_context().get_stage()

        # 配置 PhysX 关节链（Articulation）的求解器参数
        # 获取 Z1 机器人的 PhysX Articulation API
        # 注意：URDF 导入后，机器人名称可能变为小写或保持原样，需要根据实际情况调整路径
        robot_path = "/z1_description"  # Z1 机器人在 USD 场景中的路径
        
        # 尝试获取 Articulation API，如果失败则尝试其他路径
        try:
            articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, robot_path)
        except:
            # 如果失败，尝试小写路径
            robot_path = "/z1_description"
            articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, robot_path)
        
        # 设置位置求解器的迭代次数：64 次
        # 更高的迭代次数 = 更精确的物理模拟，但计算成本更高
        articulation_api.CreateSolverPositionIterationCountAttr(64)
        # 设置速度求解器的迭代次数：64 次
        # 速度迭代用于计算关节速度，影响运动的平滑度
        articulation_api.CreateSolverVelocityIterationCountAttr(64)

        # 获取每个关节的驱动器 API
        # UsdPhysics.DriveAPI.Get() 获取指定关节的驱动器 API
        # 第二个参数 "angular" 表示这是角速度/角度驱动器（用于旋转关节）
        # Z1 的所有关节都是旋转关节，所以都使用 "angular" 类型
        
        # 尝试不同的路径格式来获取关节
        # 方式1：尝试 /z1_description/joints/joint1 格式
        try:
            self.joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/joint1"), "angular")
            self.joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/joint2"), "angular")
            self.joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/joint3"), "angular")
            self.joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/joint4"), "angular")
            self.joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/joint5"), "angular")
            self.joint_6 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/joint6"), "angular")
            self.joint_gripper = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joints/jointGripper"), "angular")
            print(f"成功获取关节，使用路径格式: {robot_path}/joints/jointX")
        except:
            # 方式2：尝试 /z1_description/joint1 格式（关节直接在机器人路径下）
            try:
                self.joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joint1"), "angular")
                self.joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joint2"), "angular")
                self.joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joint3"), "angular")
                self.joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joint4"), "angular")
                self.joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joint5"), "angular")
                self.joint_6 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/joint6"), "angular")
                self.joint_gripper = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{robot_path}/jointGripper"), "angular")
                print(f"成功获取关节，使用路径格式: {robot_path}/jointX")
            except Exception as e:
                print(f"错误：无法获取关节驱动器 API: {e}")
                print("请检查关节路径是否正确")
                return

        # 为每个关节设置驱动器参数
        # set_drive_parameters() 函数参数说明：
        # 参数 1: 驱动器对象
        # 参数 2: 控制模式（"position" = 位置控制，"velocity" = 速度控制）
        # 参数 3: 目标值（位置模式下是目标角度，单位：弧度）
        # 参数 4: 刚度（stiffness，单位：N·m/rad），控制关节回到目标位置的强度
        # 参数 5: 阻尼（damping，单位：N·m·s/rad），控制关节运动的阻力
        # 参数 6: 最大力（max_force，单位：N·m），限制驱动器能施加的最大力矩
        
        # math.radians(1e8) = 100,000,000 弧度（非常高的刚度，使关节快速到达目标位置）
        # math.radians(5e7) = 50,000,000 弧度（高阻尼，使运动平滑，减少震荡）
        
        # 配置所有 6 个主要关节到初始位置（0 度），使用高刚度和高阻尼
        # 注意：刚度和阻尼参数仍然使用弧度值
        # note:joint1 lower limit = -150, upper limit = 150
        #      joint2 lower limit = -0,   upper limit = 170
        #      joint3 lower limit = -165, upper limit = 0
        #      joint4 lower limit = -187, upper limit = 187
        #      joint5 lower limit = -77,  upper limit = 77
        #      joint6 lower limit = -160, upper limit = 160
        #      jointGripper lower limit = -90, upper limit = 0.0
        set_drive_parameters(self.joint_1, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_2, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_3, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_4, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_5, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_6, "position", 0.0, math.radians(1e8), math.radians(5e7))
        
        # 配置夹爪关节到关闭位置（直接使用角度值）
        # 夹爪的刚度和阻尼可以设置得稍低一些，因为夹爪通常不需要那么强的控制
        # 注意：刚度和阻尼参数仍然使用弧度值
        set_drive_parameters(self.joint_gripper, "position", 0, math.radians(1e7), math.radians(5e6))
        
        # 初始化夹爪状态标志和目标位置
        self._gripper_open = False  # False = 关闭，True = 打开
        self._gripper_target_position = 0.0  # 初始为关闭位置（0度）

    def _on_config_drives(self):
        """
        移动到姿态按钮的点击事件处理函数
        
        这个方法将机器人的所有主要关节移动到 90 度的位置，演示如何控制机器人运动。
        在执行移动之前，它会先确保驱动器已经配置好。
        """
        # 首先调用配置方法，确保驱动器已经正确配置
        # 这样可以保证在移动之前，所有关节的刚度、阻尼等参数都已设置好
        self._on_config_robot()  # 确保驱动器首先被配置

        # 检查关节是否已正确获取
        if not hasattr(self, 'joint_1') or self.joint_1 is None:
            print("错误：关节未正确配置，请先点击 Configure 按钮")
            return

        # 将所有 6 个主要关节移动到 90 度（直接使用角度值，不需要转换为弧度）
        print("移动所有关节到 90 度位置...")
        set_drive_parameters(self.joint_1, "position", math.degrees(0)) # 关节 1 移动到 90 度
        set_drive_parameters(self.joint_2, "position", math.degrees(1.57)) # 关节 2 移动到 90 度
        set_drive_parameters(self.joint_3, "position", math.degrees(-1.57))  # 关节 3 移动到 -90 度
        set_drive_parameters(self.joint_4, "position", math.degrees(0.52))  # 关节 4 移动到 30 度
        set_drive_parameters(self.joint_5, "position", math.degrees(0.52))  # 关节 5 移动到 30 度
        set_drive_parameters(self.joint_6, "position", math.degrees(1.57))  # 关节 6 移动到 90 度
        print("所有关节已设置到 90 度位置")

    def _on_control_gripper(self):
        """
        控制夹爪按钮的点击事件处理函数
        
        这个方法切换夹爪的打开/关闭状态。
        根据 URDF 文件，夹爪关节的限制是 lower="-1.5707963267948966" upper="0.0"。
        设置为 60 度的开合范围：
        - 关闭位置：-1.57 弧度（约 -90 度，URDF 限制的下限）
        - 打开位置：-0.524 弧度（约 -30 度，从关闭位置打开 60 度）
        """
        # 首先确保驱动器已经配置
        if not hasattr(self, 'joint_gripper') or self.joint_gripper is None:
            self._on_config_robot()
        
        # 检查夹爪关节是否已正确获取
        if not hasattr(self, 'joint_gripper') or self.joint_gripper is None:
            print("错误：夹爪关节未正确配置，请先点击 Configure 按钮")
            return
        
        # 切换夹爪状态
        if self._gripper_open:
            # 如果当前是打开状态，则关闭夹爪
            # 关闭位置：0 度（对应 URDF 的 upper limit）
            self._gripper_target_position = 0.0
            set_drive_parameters(self.joint_gripper, "position", self._gripper_target_position)
            self._gripper_open = False
            print("夹爪关闭（0度）")
        else:
            # 如果当前是关闭状态，则打开夹爪
            # 打开位置：-90 度（对应 URDF 的 lower limit -1.57 rad）
            self._gripper_target_position = -90.0
            set_drive_parameters(self.joint_gripper, "position", self._gripper_target_position)
            self._gripper_open = True
            print(f"夹爪打开（-90度）")

    # ----------------------------------------------------------------------
    # RMPflow 跟随目标相关逻辑
    # ----------------------------------------------------------------------

    def _ensure_rmpflow_setup(self):
        """
        确保已为 Z1 创建 RMPflow 控制器（只初始化一次）
        """
        if self._rmp_controller is not None and self._z1_articulation is not None:
            return

        stage = omni.usd.get_context().get_stage()
        
        # 1) 创建 Z1 articulation 封装
        # 尝试检测机器人根路径
        robot_path = "/z1_description"
        if not stage.GetPrimAtPath(robot_path).IsValid():
             # 尝试查找是否存在其他可能的根节点，或默认为 "/z1_description"
             # 这里简单起见，如果默认路径无效，打印警告
             print(f"Warning: Default robot path {robot_path} not found. RMPflow might fail.")

        self._z1_articulation = SingleArticulation(prim_path=robot_path)
        
        # 尝试初始化 articulation (如果不初始化，get_joint_positions 会返回 None)
        # 注意：这通常需要在时间轴播放后才有效，但我们可以尝试调用 initialize()
        try:
            self._z1_articulation.initialize()
        except Exception as e:
            print(f"Articulation initialization warning: {e}")

        # 2) 加载我们为 Unitree_Z1 写好的 RMPflow 配置（policy_map.json 中已注册）
        rmp_cfg = mg.interface_config_loader.load_supported_motion_policy_config(
            "Unitree_Z1", "RMPflow"
        )

        # 3) 创建 RmpFlow 对象
        self._rmp_flow = mg.lula.motion_policies.RmpFlow(**rmp_cfg)

        # 4) 设置机器人基座位姿（RMPflow 需要知道 base pose）
        # 需要确保 articulation 已初始化才能获取 pose，否则使用默认值
        if self._z1_articulation.handles_initialized:
            base_pos, base_quat = self._z1_articulation.get_world_pose()
        else:
            # 默认假设在原点
            base_pos = np.zeros(3)
            base_quat = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z

        self._rmp_flow.set_robot_base_pose(
            robot_position=np.array(base_pos),
            robot_orientation=np.array(base_quat),
        )

        # 5) 将 Z1 articulation 和 RmpFlow 绑定为 ArticulationMotionPolicy
        physics_dt = 1.0 / 60.0
        art_rmp = mg.ArticulationMotionPolicy(
            robot_articulation=self._z1_articulation,
            motion_policy=self._rmp_flow,
            default_physics_dt=physics_dt,
        )

        # 6) 封装成 MotionPolicyController，后面只用 forward() 就能得到关节命令
        self._rmp_controller = mg.MotionPolicyController(
            name="z1_rmp_controller",
            articulation_motion_policy=art_rmp,
        )
        
        # 打印关节信息，方便调试
        print("=" * 60)
        print("🔧 RMPflow 控制器初始化完成")
        print(f"📋 机器人关节列表: {self._z1_articulation.dof_names}")
        print(f"🔢 总关节数: {len(self._z1_articulation.dof_names)}")
        
        # 查找并标记夹爪关节
        for i, name in enumerate(self._z1_articulation.dof_names):
            if "gripper" in name.lower():
                print(f"🔒 夹爪关节检测到: 索引 [{i}] - 名称 [{name}]")
                print(f"   此关节将被锁定，不受 RMPflow 控制")
        print("=" * 60)

    def _create_rmp_target_prim(self):
        """
        创建/获取 RMPflow 跟随的目标方块（可在视口中拖动和旋转）
        """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._rmp_target_prim_path)
        if not prim or not prim.IsValid():
            # 在 Z1 前方创建一个小方块作为末端跟随目标
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(self._rmp_target_prim_path))
            cube.CreateSizeAttr(0.05)
            
            # 添加变换操作：先旋转，后平移（符合 USD 的标准顺序）
            # 注意：操作顺序很重要！USD 按照添加顺序应用变换
            xformable = UsdGeom.Xformable(cube)
            
            # 1. 添加旋转操作（使用四元数，更直观）
            # 初始朝向：与世界坐标系对齐
            xformable.AddOrientOp(opSuffix="orient").Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))  # w, x, y, z
            
            # 2. 添加平移操作
            # 大致放在机器人前上方一点的位置
            xformable.AddTranslateOp(opSuffix="translate").Set(Gf.Vec3f(0.4, 0.0, 0.4))
            
            prim = cube.GetPrim()
            print(f"✅ 创建 RMPflow 目标: {self._rmp_target_prim_path}")
            print("💡 提示：在视口中选中目标方块，可以使用 Gizmo 工具同时调整位置和朝向")
        return prim

    def _on_rmpflow_physics_step(self, dt: float):
        """
        物理步回调：在每个 physics step 中调用 RMPflow 生成关节动作
        """
        if self._rmp_controller is None or self._z1_articulation is None:
            return

        # 检查 Articulation 是否已初始化句柄
        if not self._z1_articulation.handles_initialized:
            # 尝试初始化（如果是刚开始运行）
            try:
                self._z1_articulation.initialize()
            except:
                pass
            # 如果仍然未初始化，则本帧跳过，等待下一帧物理引擎就绪
            if not self._z1_articulation.handles_initialized:
                return

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._rmp_target_prim_path)
        if not prim or not prim.IsValid():
            return

        # 🔥 从 Xform 中读取目标位置和朝向（完整的 6D 姿态）
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        
        # 默认值
        pos = Gf.Vec3d(0.4, 0.0, 0.4)
        quat = Gf.Quatf(1.0, 0.0, 0.0, 0.0)  # w, x, y, z
        
        # 遍历所有变换操作，提取位置和旋转
        for op in ops:
            op_type = op.GetOpType()
            
            # 读取平移
            if op_type == UsdGeom.XformOp.TypeTranslate:
                pos = op.Get()
            
            # 读取旋转（四元数形式）
            elif op_type == UsdGeom.XformOp.TypeOrient:
                quat = op.Get()
            
            # 读取旋转（欧拉角形式 XYZ）
            elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
                euler = op.Get()  # 返回 (rx, ry, rz) 度数
                # 将欧拉角（度）转换为四元数
                # 注意：USD 使用度数，需要转换为弧度
                import math
                rx = math.radians(euler[0])
                ry = math.radians(euler[1])
                rz = math.radians(euler[2])
                
                # 欧拉角转四元数（ZYX 顺序）
                cy = math.cos(rz * 0.5)
                sy = math.sin(rz * 0.5)
                cp = math.cos(ry * 0.5)
                sp = math.sin(ry * 0.5)
                cr = math.cos(rx * 0.5)
                sr = math.sin(rx * 0.5)
                
                quat = Gf.Quatf(
                    cr * cp * cy + sr * sp * sy,  # w
                    sr * cp * cy - cr * sp * sy,  # x
                    cr * sp * cy + sr * cp * sy,  # y
                    cr * cp * sy - sr * sp * cy   # z
                )

        # 转换为 NumPy 数组
        target_pos = np.array([pos[0], pos[1], pos[2]], dtype=float)
        
        # 🔥 使用目标的实际朝向（四元数格式：[w, x, y, z]）
        # 注意：Isaac Sim 的四元数格式是 [w, x, y, z]
        target_ori = np.array([
            quat.GetReal(),      # w
            quat.GetImaginary()[0],  # x
            quat.GetImaginary()[1],  # y
            quat.GetImaginary()[2]   # z
        ], dtype=float)

        # 调用 RMPflow 控制器，生成关节动作（同时跟随位置和朝向）
        actions = self._rmp_controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_ori,
        )

        # 🔥 关键修复：在应用 action 之前，修改夹爪关节的命令
        # 问题根源：RMPflow 返回的 actions 包含了所有关节（包括夹爪）的控制命令
        # 解决方案：在应用之前，将夹爪关节的目标位置替换为用户设定的值
        try:
            # 获取当前关节位置，确定夹爪关节的索引
            joint_names = self._z1_articulation.dof_names
            
            # 查找夹爪关节的索引
            gripper_joint_index = None
            for i, name in enumerate(joint_names):
                if "gripper" in name.lower() or "jointgripper" in name.lower():
                    gripper_joint_index = i
                    break
            
            # 如果找到了夹爪关节
            if gripper_joint_index is not None:
                # 修改 action 中夹爪关节的目标位置
                # ArticulationAction 有 joint_positions 属性
                if hasattr(actions, 'joint_positions') and actions.joint_positions is not None:
                    # 保存原始值（用于调试）
                    original_value = actions.joint_positions[gripper_joint_index]
                    
                    # 将夹爪目标位置转换为弧度
                    import math
                    target_rad = math.radians(self._gripper_target_position)
                    
                    # 替换夹爪关节的目标位置
                    actions.joint_positions[gripper_joint_index] = target_rad
                    
                    # 调试输出
                    if self._debug_gripper_lock:
                        print(f"🔒 夹爪锁定: 索引[{gripper_joint_index}] | RMPflow={original_value:.3f} → 锁定={target_rad:.3f} rad ({self._gripper_target_position}°)")
        except Exception as e:
            # 如果修改失败，打印警告但不影响主要控制
            print(f"⚠️ [WARNING] 无法锁定夹爪关节: {e}")
            import traceback
            traceback.print_exc()
            pass

        # 应用修改后的 RMPflow 动作到 Z1 articulation
        self._z1_articulation.get_articulation_controller().apply_action(actions)

    def _on_toggle_debug(self):
        """
        切换调试模式按钮的回调函数
        启用后，会在控制台实时显示夹爪锁定信息
        """
        # 切换调试标志
        self._debug_gripper_lock = not self._debug_gripper_lock
        
        # 打印状态
        if self._debug_gripper_lock:
            print("\n" + "=" * 60)
            print("🐛 调试模式已启用")
            print("   控制台将显示夹爪锁定详细信息")
            print("   旋转目标方块，观察 RMPflow 值 vs 锁定值")
            print("=" * 60 + "\n")
        else:
            print("\n" + "=" * 60)
            print("✅ 调试模式已关闭")
            print("=" * 60 + "\n")

    def _on_reconfigure_camera(self):
        """
        重新配置相机按钮的回调函数
        删除现有相机并重新创建（通常不需要，相机会自动配置）
        """
        print("\n🔄 正在重新配置相机...")
        # 直接走 force_create 流程：会清理所有可能遗留的相机节点并重建
        self._setup_d435i_camera(force_create=True)
    
    def _setup_d435i_camera(self, force_create: bool = False):
        """
        内部方法：配置 D435i 相机传感器
        可以被按钮调用，也可以在加载时自动调用
        """
        print("\n" + "=" * 60)
        print("📷 开始配置 D435i 相机传感器...")
        print("=" * 60)
        
        stage = omni.usd.get_context().get_stage()
        robot_path = "/z1_description"
        
        # 查找 camera_link 的路径（由于 merge_fixed_joints=True，相机坐标系都合并了）
        camera_link_path = None
        possible_paths = [
            f"{robot_path}/camera_link",
            f"{robot_path}/camera_link/camera_link",  # 有时会嵌套
        ]
        
        for path in possible_paths:
            if stage.GetPrimAtPath(path).IsValid():
                camera_link_path = path
                break
        
        if not camera_link_path:
            print("❌ 错误：未找到 camera_link")
            print("   请确保已经加载了带相机的机器人")
            return
        
        print(f"✅ 找到相机 Link: {camera_link_path}")
        
        # 清理可能的错误位置相机（早期遗留，例如直接挂在 z1_description/Camera）
        stray_cam_paths = [
            f"{robot_path}/Camera",
            f"{robot_path}/camera",
            f"{camera_link_path}/Camera",
            f"{camera_link_path}/camera",
            f"{camera_link_path}/rgb_camera",
            f"{camera_link_path}/depth_camera",
            f"{camera_link_path}/camera_color_optical_frame/rgb_camera",
            f"{camera_link_path}/camera_depth_optical_frame/depth_camera",
        ]
        if force_create:
            for p in stray_cam_paths:
                if stage.GetPrimAtPath(p).IsValid():
                    stage.RemovePrim(p)
                    print(f"⚠️  删除遗留的相机节点: {p}")

        # 选择父坐标系：优先使用光学坐标系，保证与 D435i 对齐
        color_optical_parent = None
        depth_optical_parent = None
        for path in [
            f"{camera_link_path}/camera_color_optical_frame",
            f"{camera_link_path}/camera_color_frame",
            camera_link_path,
        ]:
            if stage.GetPrimAtPath(path).IsValid():
                color_optical_parent = path
                break
        for path in [
            f"{camera_link_path}/camera_depth_optical_frame",
            f"{camera_link_path}/camera_depth_frame",
            camera_link_path,
        ]:
            if stage.GetPrimAtPath(path).IsValid():
                depth_optical_parent = path
                break

        # 如果相机已存在且不强制重建，则跳过
        rgb_camera_path = f"{color_optical_parent}/rgb_camera"
        depth_camera_path = f"{depth_optical_parent}/depth_camera"
        if stage.GetPrimAtPath(rgb_camera_path).IsValid() and not force_create:
            print("⚠️  相机已存在，跳过创建（如需重建请点击 Reconfigure Camera）")
            return
        
        # Intel RealSense D435i 的实际参数
        # RGB 相机参数
        rgb_width = 1280
        rgb_height = 720
        rgb_horizontal_fov = 69.4  # 度
        
        # 深度相机参数
        depth_width = 1280
        depth_height = 720
        depth_horizontal_fov = 87.0  # 度（深度相机 FOV 更大）
        depth_min_range = 0.105  # 最小深度 10.5cm
        depth_max_range = 10.0   # 最大深度 10m

        # ------------------------------------------------------------------
        # 成像参数（USD 单位为 mm）：用 FOV 推导 focalLength/horizontalAperture
        # USD 定义：hfov = 2 * atan((horizontalAperture/2) / focalLength)
        #
        # 这里我们选一个“合理的”水平光阑尺寸（mm），再算 focalLength，使得 FOV 匹配。
        # 重要的是比例（aperture/focalLength），绝对值对视觉渲染影响不大。
        # ------------------------------------------------------------------
        horizontal_aperture_mm = 20.955  # USD 常用默认值（mm）

        def _compute_focal_length_mm(hfov_deg: float) -> float:
            return (horizontal_aperture_mm * 0.5) / math.tan(math.radians(hfov_deg * 0.5))

        def _vertical_aperture_mm(width_px: int, height_px: int) -> float:
            return horizontal_aperture_mm * (float(height_px) / float(width_px))

        def _set_or_create_rotate_xyz_deg(prim, rot_deg: Gf.Vec3f, suffix: str = "offset"):
            """
            只设置 Camera Prim 自己的旋转，不添加任何平移，以确保：
            - rgb/depth 相机与 D435i（optical_frame）相对位置/距离不变
            - 两个相机相对姿态一致（同一组 rot）
            """
            xformable = UsdGeom.Xformable(prim)
            # 若已经存在 RotateXYZ op（同 suffix），就直接覆盖
            try:
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ and op.GetOpName().endswith(f":{suffix}"):
                        op.Set(rot_deg)
                        return
            except Exception:
                pass
            # 否则创建一个 RotateXYZ op
            xformable.AddRotateXYZOp(opSuffix=suffix).Set(rot_deg)
        
        try:
            # 1. 创建 RGB 相机
            print("\n📸 配置 RGB 相机...")
            rgb_camera_path = f"{color_optical_parent}/rgb_camera"
            
            # 创建相机 prim，挂在光学坐标系下，确保朝向与 D435i 一致
            rgb_camera_prim = stage.DefinePrim(rgb_camera_path, "Camera")
            rgb_camera = UsdGeom.Camera(rgb_camera_prim)

            # ✅ 将相机可见性设为不可见（不影响相机用于渲染/取图）
            try:
                UsdGeom.Imageable(rgb_camera_prim).MakeInvisible()
            except Exception:
                pass

            # ✅ 固化你确认的最终角度（度）：(90, 0, -90)
            # 仅改旋转，不改平移 => 相对位置/距离保持不变
            _set_or_create_rotate_xyz_deg(rgb_camera_prim, Gf.Vec3f(90.0, 0.0, -90.0))

            # 设置相机参数（mm）
            rgb_focal_length_mm = _compute_focal_length_mm(rgb_horizontal_fov)
            rgb_camera.CreateFocalLengthAttr(rgb_focal_length_mm)
            rgb_camera.CreateHorizontalApertureAttr(horizontal_aperture_mm)
            rgb_camera.CreateVerticalApertureAttr(_vertical_aperture_mm(rgb_width, rgb_height))
            rgb_camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))  # 近裁剪面和远裁剪面
            
            print(f"   ✅ RGB 相机创建成功")
            print(f"      分辨率: {rgb_width}x{rgb_height}")
            print(f"      水平 FOV: {rgb_horizontal_fov}°")
            print(f"      焦距: {rgb_focal_length_mm:.3f}mm")
            
            # 2. 创建深度相机
            print("\n🎯 配置深度相机...")
            depth_camera_path = f"{depth_optical_parent}/depth_camera"
            
            # 创建深度相机 prim，挂在深度光学坐标系下
            depth_camera_prim = stage.DefinePrim(depth_camera_path, "Camera")
            depth_camera = UsdGeom.Camera(depth_camera_prim)

            # ✅ 将相机可见性设为不可见（不影响相机用于渲染/取图）
            try:
                UsdGeom.Imageable(depth_camera_prim).MakeInvisible()
            except Exception:
                pass

            # ✅ 固化你确认的最终角度（度）：(90, 0, -90)
            _set_or_create_rotate_xyz_deg(depth_camera_prim, Gf.Vec3f(90.0, 0.0, -90.0))

            # 设置深度相机属性（mm）
            depth_focal_length_mm = _compute_focal_length_mm(depth_horizontal_fov)
            depth_camera.CreateFocalLengthAttr(depth_focal_length_mm)
            depth_camera.CreateHorizontalApertureAttr(horizontal_aperture_mm)
            depth_camera.CreateVerticalApertureAttr(_vertical_aperture_mm(depth_width, depth_height))
            depth_camera.CreateClippingRangeAttr(Gf.Vec2f(depth_min_range, depth_max_range))

            # 启动“每 2 秒打印一次相机旋转”的后台任务（只启动一次）
            self._rgb_camera_prim_path = rgb_camera_path
            self._depth_camera_prim_path = depth_camera_path
            if self._camera_pose_print_task is None:
                self._camera_pose_print_task = asyncio.ensure_future(self._camera_pose_printer_loop())
            
            print(f"   ✅ 深度相机创建成功")
            print(f"      分辨率: {depth_width}x{depth_height}")
            print(f"      水平 FOV: {depth_horizontal_fov}°")
            print(f"      焦距: {depth_focal_length_mm:.3f}mm")
            print(f"      深度范围: {depth_min_range}m - {depth_max_range}m")
            
            # 3. 相机已创建完成，无需额外的渲染配置
            # Isaac Sim 会自动识别 Camera prim 并支持渲染
            
            # 4. 启用相机视锥体可视化
            print("\n👁️ 启用相机视锥体可视化...")
            try:
                # RGB 相机视锥体
                UsdGeom.Camera(rgb_camera_prim).CreatePurposeAttr("render")
                
                # 深度相机视锥体
                UsdGeom.Camera(depth_camera_prim).CreatePurposeAttr("render")
                
                print(f"   ✅ 视锥体可视化已启用")
                print(f"      在视口中选择相机可以看到视锥体")
                
            except Exception as e:
                print(f"   ⚠️ 视锥体可视化失败: {e}")
            
            print("\n" + "=" * 60)
            print("✅ D435i 相机配置完成！")
            print("\n💡 使用提示：")
            print("   【查看相机视图】")
            print("   1. 在场景树中选择 'rgb_camera' 或 'depth_camera'")
            print("   2. 点击视口右上角的相机图标 📷")
            print("   3. 或者右键相机 → 'Set as Active Camera'")
            print("")
            print("   【获取相机数据】")
            print("   注意：USD 中并不存在“深度相机”这种不同的相机 Prim 类型；")
            print("         depth_camera 仍然是 Camera Prim，深度由渲染输出的 depth AOV 决定。")
            print("         我们在这里主要通过 ClippingRange 来匹配 D435i 的深度量程。")
            print("   方法 1 (Python):")
            print("      from omni.isaac.sensor import Camera")
            print(f"      rgb_cam = Camera('{rgb_camera_path}')")
            print("      rgb_cam.initialize()")
            print("      frame = rgb_cam.get_rgba()")
            print("")
            print("   方法 2 (ROS2):")
            print("      使用 isaac_ros_image_pipeline 包")
            print("")
            print("   方法 3 (Replicator):")
            print("      使用 Isaac Sim 的 Replicator 进行数据采集")
            print("=" * 60 + "\n")
            
        except Exception as e:
            print(f"\n❌ 相机配置失败: {e}")
            import traceback
            traceback.print_exc()
            print("=" * 60 + "\n")

    def _on_rmpflow_follow_button(self):
        """
        "RMPflow Follow" 按钮回调：
        1. 确保 RMPflow 控制器已经创建
        2. 创建/获取目标方块
        3. 订阅物理步事件，在每个 step 中调用 RMPflow
        """
        # 确保夹爪目标位置已初始化（防止 RMPflow 控制夹爪）
        if not hasattr(self, '_gripper_target_position'):
            self._gripper_target_position = 0.0  # 默认关闭位置
        if not hasattr(self, '_gripper_open'):
            self._gripper_open = False
        if not hasattr(self, '_debug_gripper_lock'):
            self._debug_gripper_lock = False  # 默认关闭调试模式
        
        # 确保 RMPflow 控制器已初始化
        self._ensure_rmpflow_setup()
        # 创建跟随目标
        self._create_rmp_target_prim()

        # 启动物理步回调（只注册一次）
        if self._rmp_physx_subscription is None:
            physx_interface = physx.get_physx_interface()
            self._rmp_physx_subscription = physx_interface.subscribe_physics_step_events(
                self._on_rmpflow_physics_step
            )

        # 确保时间轴在播放（如果你用的是 GUI，可以点 Play；这里再保险启动一次）
        try:
            timeline = omni.timeline.get_timeline()
            if not timeline.is_playing():
                timeline.play()
        except Exception:
            # 如果 timeline 不可用，忽略，让用户手动点击 Play
            pass

