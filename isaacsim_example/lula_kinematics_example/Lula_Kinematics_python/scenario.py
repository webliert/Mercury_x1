# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION 及其许可方保留本软件、相关文档及其任何修改的所有知识产权和专有权利。
# 未经 NVIDIA CORPORATION 明确许可协议，严禁使用、复制、披露或分发本软件及相关文档。
#

import os
import omni.usd

import carb
import numpy as np
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
    interface_config_loader,
)


class FrankaKinematicsExample:
    def __init__(self):
        self._kinematics_solver = None
        self._articulation_kinematics_solver = None
        self._articulation = None
        self._target = None
        self._ik_warned = False  # 新增：逆解警告标志
        self._update_called = False  # 新增：只调用一次的标志

    def load_example_assets(self):
        # 将 Franka 机器人和目标添加到场景中
        robot_prim_path = "/World/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        # 等待 prim 加载完成，再实例化Articulation，
        # add by lxy,现象：点击run后机械臂就下坠，并且不断输出不能获取关节位置、未收敛到解
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            print("Prim path:", prim.GetPath())
        import time
        max_wait = 100  # 最多等1秒
        for i in range(max_wait):
            prim = stage.GetPrimAtPath(robot_prim_path)
            if prim.IsValid():
                articulation = Articulation(robot_prim_path)
                if hasattr(articulation, "get_joints"):
                    num_joints = len(articulation.get_joints())
                elif hasattr(articulation, "get_joint_count"):
                    num_joints = articulation.get_joint_count()
                else:
                    num_joints = 0
                print(f"Waiting... joints: {num_joints} (try {i})")
                if num_joints > 0:
                    break
            time.sleep(0.01)

        print("Prim valid:", prim.IsValid())
        print("Articulation joints:", num_joints)
        print(dir(articulation))
        self._articulation = articulation

        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim("/World/target", scale=[0.04, 0.04, 0.04])
        self._target.set_default_state(np.array([0.3, 0, 0.5]), euler_angles_to_quats([0, np.pi, 0]))

        # 返回添加到场景中的资产，以便可以在 core.World 中注册
        return self._articulation, self._target

    def setup(self):
        # 为该机器人加载 URDF 和 Lula 机器人描述文件
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        kinematics_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")

        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=kinematics_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
            urdf_path=kinematics_config_dir + "/franka/lula_franka_gen.urdf",
        )

        # 支持的机器人可以用更简单的方式加载运动学
        # print("Supported Robots with a Lula Kinematics Config:", interface_config_loader.get_supported_robots_with_lula_kinematics())
        # kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
        # self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)

        print("可以用于运动学计算的有效坐标系名称:", self._kinematics_solver.get_all_frame_names())

        end_effector_name = "right_gripper"
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._articulation, self._kinematics_solver, end_effector_name
        )

    def update(self, step: float):
        if self._update_called:
            return  # 已经调用过一次，直接返回
        self._update_called = True

        # 获取目标的位置和姿态
        target_position, target_orientation = self._target.get_world_pose()

        # 跟踪机器人基座的任何移动
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        # 计算逆运动学，获得动作和是否成功
        action, success = self._articulation_kinematics_solver.compute_inverse_kinematics(
            target_position, target_orientation
        )

        if success:
            self._articulation.apply_action(action)
            if self._ik_warned:
                carb.log_warn("逆运动学已恢复正常")
            self._ik_warned = False  # 一旦成功，重置警告标志
        else:
            if not self._ik_warned:
                carb.log_warn("逆运动学未收敛到解，未执行任何动作")
                self._ik_warned = True

# 未使用的正向运动学示例:
        # ee_position,ee_rot_mat = articulation_kinematics_solver.compute_end_effector_pose()

    def reset(self):
        # 运动学是无状态的
        pass
