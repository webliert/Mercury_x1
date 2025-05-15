# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.stage import create_new_stage, get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux

from .scenario import FrankaKinematicsExample


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        # Frames 是可以包含多个 UI 元素的子窗口
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        # 使用 UIElementWrapper 实例创建的 UI 元素
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        # 获取时间轴接口，以便以编程方式控制停止/暂停/播放
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        # 运行示例的初始化
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    #           下面的函数会被 extension.py 自动调用
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        # 当从工具栏打开 UI 时的回调。
        # 该函数会在 build_ui() 之后被直接调用。
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        # 时间轴事件（播放、暂停、停止）的回调
        # 参数 event (omni.timeline.TimelineEventType): 事件类型
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
            # For complete robustness, the user should resolve those edge cases here
            # In general, for extensions based off this template, there is no value to having the user click the play/stop
            # button instead of using the Load/Reset/Run buttons provided.
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False
            # 当用户通过 UI 点击停止按钮时，可能会遇到一些边界情况导致出错
            # 为了健壮性，用户应在这里处理这些边界情况
            # 通常基于此模板的扩展，建议只用 Load/Reset/Run 按钮而不是直接用播放/停止按钮

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        # 物理步进的回调
        # 只有时间轴在播放时才会发生物理步进
        # 参数 step (float): 物理步长
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        # 场景事件的回调
        # 参数 event (omni.usd.StageEventType): 事件类型
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            # 如果用户打开了新场景，扩展应完全重置
            self._reset_extension()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        # 当场景关闭或扩展热重载时调用
        # 执行必要的清理操作，比如移除激活的回调函数
        # 从 isaacsim.gui.components.element_wrappers 导入的按钮实现了 cleanup 方法，应调用
        for ui_elem in self.wrapped_ui_elements:
            if hasattr(ui_elem, "cleanup"):
                ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        # 构建自定义 UI 工具以运行你的扩展
        # 每当 UI 窗口关闭并重新打开时都会调用此函数
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # 替换 LoadButton
                self._load_btn = ui.Button(
                    "load",
                    clicked_fn=self._on_load_clicked
                )
                # 替换 ResetButton
                self._reset_btn = ui.Button(
                    "reset",
                    clicked_fn=self._on_reset_clicked,
                    enabled=False
                )
                self.wrapped_ui_elements.append(self._load_btn)
                self.wrapped_ui_elements.append(self._reset_btn)

        run_scenario_frame = CollapsableFrame("Run Scenario")

        with run_scenario_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._on_run_scenario_a_text,
                    on_b_click_fn=self._on_run_scenario_b_text,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

    # 新增按钮回调
    def _on_load_clicked(self):
        self._setup_scene()
        self._setup_scenario()
        self._reset_btn.enabled = True

    def _on_reset_clicked(self):
        self._on_post_reset_btn()

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    # 以下函数为示例支持函数，可根据需要删除或替换
    ######################################################################################

    def _on_init(self):
        self._articulation = None
        self._cuboid = None
        self._scenario = FrankaKinematicsExample()

    def _add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        # 新建的 stage 默认没有光源。此函数创建一个球形光源
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

    def _setup_scene(self):
        """
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.
        """
        # 此函数作为 setup_scene_fn 回调绑定到 Load 按钮
        # 按下 Load 按钮时，会创建新的 World 实例并调用此函数
        # 用户应在此加载资产到场景，并添加到 World.Scene
        create_new_stage()
        self._add_light_to_stage()
        set_camera_view(eye=[1.5, 1.25, 2], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        loaded_objects = self._scenario.load_example_assets()

        # Add user-loaded objects to the World
        # 将用户加载的对象添加到 World
        world = World.instance()
        if world is None:
            world = World()
        for loaded_object in loaded_objects:
            world.scene.add(loaded_object)

    def _setup_scenario(self):
        """
        This function is attached to the Load Button as the setup_post_load_fn callback.
        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.

        In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
        provided prim in a circle around the robot.
        """
        # 此函数作为 setup_post_load_fn 回调绑定到 Load 按钮
        # 用户可以假设资产已通过 setup_scene_fn 加载，对象已初始化，时间轴暂停在第0帧
        # 本例中，初始化一个场景，循环移动每个机器人关节，同时让物体绕机器人转圈
        self._scenario.setup()

        # UI management
        # UI 管理
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        # 此函数作为 post_reset_fn 回调绑定到 Reset 按钮
        # 用户可以假设对象已初始化，时间轴暂停在第0帧
        # 也可以假设添加到 World.Scene 的对象已回到默认位置
        # 例如 cube 会回到 setup_scene 创建时的位置
        self._scenario.reset()

        # UI management
        # UI 管理
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """This function is attached to the Run Scenario StateButton.
        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        Args:
            step (float): The dt of the current physics step
        """
        # 此函数作为 Run Scenario StateButton 的 physics_callback_fn 回调
        # 当点击 "RUN" 时，每个物理步都会调用此函数
        # 当点击 "STOP" 时，物理回调会被移除
        # 参数 step (float): 当前物理步的 dt
        self._scenario.update(step)

    def _on_run_scenario_a_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_a_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "RUN".

        This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
        the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
        through the left-hand UI toolbar.
        """
        # 此函数作为 Run Scenario StateButton 的 on_a_click_fn 回调
        # 当按钮显示 "RUN" 时点击会调用
        # 该函数会播放时间轴，开始物理步进
        # 加载或重置后时间轴会暂停，直到用户手动或编程方式播放
        self._timeline.play()

    def _on_run_scenario_b_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_b_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "STOP"

        Pausing the timeline on b_text is not strictly necessary for this example to run.
        Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
        the robot will stop getting new commands and the cube will stop updating without needing to
        pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
        forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
        this example prettier, but if curious, the user should observe what happens when this line is removed.
        """
        # 此函数作为 Run Scenario StateButton 的 on_b_click_fn 回调
        # 当按钮显示 "STOP" 时点击会调用
        # 暂停时间轴并不是必须的，点击 "STOP" 会取消物理回调，机器人和物体会停止更新
        # 暂停时间轴可以防止机器人因动量继续移动几帧，使演示效果更好
        self._timeline.pause()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        # 当用户打开新场景时调用，所有状态应被重置
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
        # 重置 UI 状态
