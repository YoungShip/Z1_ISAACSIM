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

import asyncio
import math
import weakref
import os
import json
import omni
import omni.ui as ui
import omni.physx as physx
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics, UsdGeom
import numpy as np
from .common import set_drive_parameters

EXTENSION_NAME = "Import Z1"

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._window = None

        self._z1_articulation: SingleArticulation | None = None
        self._rmp_flow = None
        self._rmp_controller: mg.MotionPolicyController | None = None
        self._rmp_physx_subscription = None
        self._rmp_target_prim_path = "/World/Z1_RMP_Target"

        self.example_name = "Z1 URDF"
        self.category = "Import Robots"

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=self._build_window,
            ui_hook=self._build_ui,
            category=self.category,
        )

    def _build_window(self):
        pass

    def _build_ui(self):
        with ui.VStack(spacing=5, height=0):
            title = "Import a Z1 Robot via URDF"
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html"
            overview = "This Example shows you import a Unitree Z1 robot arm with gripper via URDF.\n\nPress the 'Open in IDE' button to view the source code."
            setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

            frame = ui.CollapsableFrame(
                title="Command Panel",
                height=0,
                collapsed=False,
                style=get_style(),
                style_type_name_override="CollapsableFrame",
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )
            with frame:
                with ui.VStack(style=get_style(), spacing=5):
                    dict = {
                        "label": "Load Robot",
                        "type": "button",
                        "text": "Load",
                        "tooltip": "Load a Z1 Robot with Gripper into the Scene",
                        "on_clicked_fn": self._on_load_robot,
                    }
                    btn_builder(**dict)

                    dict = {
                        "label": "Configure Drives",
                        "type": "button",
                        "text": "Configure",
                        "tooltip": "Configure Joint Drives",
                        "on_clicked_fn": self._on_config_robot,
                    }
                    btn_builder(**dict)

                    dict = {
                        "label": "Move to Pose",
                        "type": "button",
                        "text": "Move",
                        "tooltip": "Drive the Robot to a specific pose",
                        "on_clicked_fn": self._on_config_drives,
                    }
                    btn_builder(**dict)

                    dict = {
                        "label": "Control Gripper",
                        "type": "button",
                        "text": "Open/Close Gripper",
                        "tooltip": "Open or close the gripper",
                        "on_clicked_fn": self._on_control_gripper,
                    }
                    btn_builder(**dict)

                    dict = {
                        "label": "RMPflow Follow Target",
                        "type": "button",
                        "text": "RMPflow Follow",
                        "tooltip": "Use RMPflow to make Z1 end-effector follow a target with collision avoidance",
                        "on_clicked_fn": self._on_rmpflow_follow_button,
                    }
                    btn_builder(**dict)

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        self._window = None
        self._rmp_physx_subscription = None
        self._rmp_controller = None
        self._z1_articulation = None

    def _menu_callback(self):
        if self._window is None:
            self._build_ui()
        self._window.visible = not self._window.visible

    def _on_load_robot(self):
        load_stage = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._load_robot(load_stage))

    async def _load_robot(self, task):
        done, pending = await asyncio.wait({task})
        if task in done:
            status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            import_config.merge_fixed_joints = False
            import_config.fix_base = True
            import_config.make_default_prim = True
            import_config.create_physics_scene = True
            
            # Resolve relative path to URDF
            # __file__ is in exts/isaacsim.import_z1/isaacsim/import_z1/import_z1.py
            # We need to go up 3 levels to get to exts/isaacsim.import_z1
            ext_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            urdf_path = os.path.join(ext_path, "data/z1_description/urdf/z1_with_gripper.urdf")
            urdf_path = os.path.abspath(urdf_path)

            omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=urdf_path,
                import_config=import_config,
            )

            camera_state = ViewportCameraState("/OmniverseKit_Persp")
            camera_state.set_position_world(Gf.Vec3d(1.5, -1.5, 0.8), True)
            camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.0), True)

            stage = omni.usd.get_context().get_stage()
            scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)

            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(500)
            distantLight.CreateAngleAttr().Set(-150)

    def _on_config_robot(self):
        stage = omni.usd.get_context().get_stage()
        robot_path = "/z1_description"
        
        try:
            articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, robot_path)
        except:
            robot_path = "/z1_description"
            articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, robot_path)
        
        articulation_api.CreateSolverPositionIterationCountAttr(64)
        articulation_api.CreateSolverVelocityIterationCountAttr(64)

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
                return

        set_drive_parameters(self.joint_1, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_2, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_3, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_4, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_5, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_6, "position", 0.0, math.radians(1e8), math.radians(5e7))
        set_drive_parameters(self.joint_gripper, "position", 0, math.radians(1e7), math.radians(5e6))
        self._gripper_open = False

    def _on_config_drives(self):
        self._on_config_robot()
        if not hasattr(self, 'joint_1') or self.joint_1 is None:
            print("错误：关节未正确配置，请先点击 Configure 按钮")
            return

        print("移动所有关节到 90 度位置...")
        set_drive_parameters(self.joint_1, "position", math.degrees(0))
        set_drive_parameters(self.joint_2, "position", math.degrees(1.57))
        set_drive_parameters(self.joint_3, "position", math.degrees(-1.57))
        set_drive_parameters(self.joint_4, "position", math.degrees(0.52))
        set_drive_parameters(self.joint_5, "position", math.degrees(0.52))
        set_drive_parameters(self.joint_6, "position", math.degrees(1.57))
        print("所有关节已设置到 90 度位置")

    def _on_control_gripper(self):
        if not hasattr(self, 'joint_gripper') or self.joint_gripper is None:
            self._on_config_robot()
        
        if not hasattr(self, 'joint_gripper') or self.joint_gripper is None:
            print("错误：夹爪关节未正确配置，请先点击 Configure 按钮")
            return
        
        if self._gripper_open:
            set_drive_parameters(self.joint_gripper, "position", 2)
            self._gripper_open = False
            print("夹爪关闭（-90度）")
        else:
            set_drive_parameters(self.joint_gripper, "position", -90)
            self._gripper_open = True
            print(f"夹爪打开（-30度，开合范围60度）")

    def _ensure_rmpflow_setup(self):
        if self._rmp_controller is not None and self._z1_articulation is not None:
            return

        stage = omni.usd.get_context().get_stage()
        robot_path = "/z1_description"
        if not stage.GetPrimAtPath(robot_path).IsValid():
             print(f"Warning: Default robot path {robot_path} not found. RMPflow might fail.")

        self._z1_articulation = SingleArticulation(prim_path=robot_path)
        try:
            self._z1_articulation.initialize()
        except Exception as e:
            print(f"Articulation initialization warning: {e}")

        # Load RMPflow config from local files
        module_dir = os.path.dirname(__file__)
        rmp_config_dir = os.path.join(module_dir, "rmpflow")
        config_path = os.path.join(rmp_config_dir, "config.json")
        
        with open(config_path, 'r') as f:
            config_data = json.load(f)
            
        relative_paths = config_data.get("relative_asset_paths", {})
        rmp_cfg = {
            "robot_description_path": os.path.join(rmp_config_dir, relative_paths["robot_description_path"]),
            "urdf_path": os.path.join(rmp_config_dir, relative_paths["urdf_path"]),
            "rmpflow_config_path": os.path.join(rmp_config_dir, relative_paths["rmpflow_config_path"]),
            "end_effector_frame_name": config_data["end_effector_frame_name"],
            "maximum_substep_size": config_data["maximum_substep_size"],
            "ignore_robot_state_updates": config_data.get("ignore_robot_state_updates", False)
        }

        self._rmp_flow = mg.lula.motion_policies.RmpFlow(**rmp_cfg)

        if self._z1_articulation.handles_initialized:
            base_pos, base_quat = self._z1_articulation.get_world_pose()
        else:
            base_pos = np.zeros(3)
            base_quat = np.array([1.0, 0.0, 0.0, 0.0])

        self._rmp_flow.set_robot_base_pose(
            robot_position=np.array(base_pos),
            robot_orientation=np.array(base_quat),
        )

        physics_dt = 1.0 / 60.0
        art_rmp = mg.ArticulationMotionPolicy(
            robot_articulation=self._z1_articulation,
            motion_policy=self._rmp_flow,
            default_physics_dt=physics_dt,
        )

        self._rmp_controller = mg.MotionPolicyController(
            name="z1_rmp_controller",
            articulation_motion_policy=art_rmp,
        )

    def _create_rmp_target_prim(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._rmp_target_prim_path)
        if not prim or not prim.IsValid():
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(self._rmp_target_prim_path))
            cube.CreateSizeAttr(0.05)
            cube.AddTranslateOp().Set(Gf.Vec3f(0.4, 0.0, 0.4))
            prim = cube.GetPrim()
        return prim

    def _on_rmpflow_physics_step(self, dt: float):
        if self._rmp_controller is None or self._z1_articulation is None:
            return

        if not self._z1_articulation.handles_initialized:
            try:
                self._z1_articulation.initialize()
            except:
                pass
            if not self._z1_articulation.handles_initialized:
                return

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._rmp_target_prim_path)
        if not prim or not prim.IsValid():
            return

        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        pos = Gf.Vec3d(0.4, 0.0, 0.4)
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                pos = op.Get()
                break

        target_pos = np.array([pos[0], pos[1], pos[2]], dtype=float)
        target_ori = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        actions = self._rmp_controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_ori,
        )

        self._z1_articulation.get_articulation_controller().apply_action(actions)

    def _on_rmpflow_follow_button(self):
        self._ensure_rmpflow_setup()
        self._create_rmp_target_prim()

        if self._rmp_physx_subscription is None:
            physx_interface = physx.get_physx_interface()
            self._rmp_physx_subscription = physx_interface.subscribe_physics_step_events(
                self._on_rmpflow_physics_step
            )

        try:
            timeline = omni.timeline.get_timeline()
            if not timeline.is_playing():
                timeline.play()
        except Exception:
            pass

