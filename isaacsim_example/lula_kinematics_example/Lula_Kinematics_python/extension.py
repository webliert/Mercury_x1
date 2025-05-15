# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc

import omni
import omni.ext
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .global_variables import EXTENSION_DESCRIPTION, EXTENSION_TITLE
from .ui_builder import UIBuilder

"""
This file serves as a basic template for the standard boilerplate operations
that make a UI-based extension appear on the toolbar.
# 此文件作为标准样板，帮助扩展在工具栏上以UI形式出现

This implementation is meant to cover most use-cases without modification.
# 该实现覆盖了大多数使用场景，无需修改

Various callbacks are hooked up to a seperate class UIBuilder in .ui_builder.py
# 各种回调被连接到 .ui_builder.py 文件中的 UIBuilder 类

Most users will be able to make their desired UI extension by interacting solely with
UIBuilder.
# 大多数用户只需与 UIBuilder 交互即可实现所需的 UI 扩展

This class sets up standard useful callback functions in UIBuilder:
    on_menu_callback: Called when extension is opened
    on_timeline_event: Called when timeline is stopped, paused, or played
    on_physics_step: Called on every physics step
    on_stage_event: Called when stage is opened or closed
    cleanup: Called when resources such as physics subscriptions should be cleaned up
    build_ui: User function that creates the UI they want.
# 该类在 UIBuilder 中设置了标准且有用的回调函数：
#   on_menu_callback: 扩展被打开时调用
#   on_timeline_event: 时间轴停止、暂停或播放时调用
#   on_physics_step: 每个物理步调用
#   on_stage_event: 场景打开或关闭时调用
#   cleanup: 清理物理订阅等资源时调用
#   build_ui: 用户自定义UI的函数
"""

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        # 初始化扩展和 UI 元素

        print("Extension started")
        print("Lula_Kinematics_python extension.py loaded")
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Window
        # 构建窗口
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        # 注册菜单项到工具栏
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)
        # 添加菜单项到工具栏

        # Filled in with User Functions
        # 用用户自定义函数填充
        self.ui_builder = UIBuilder()

        # Events
        # 事件相关
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        print("Extension shutdown")
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)
        # 从工具栏移除菜单项

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")
        # 注销菜单动作

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        # 清理 UIBuilder 资源
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            # 订阅场景和时间轴事件
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()
            # 清理 UIBuilder 资源

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()
        # 构建扩展 UI

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()
        # 异步将窗口停靠到指定位置

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    # 以下函数调用用户自定义函数
    #################################################################

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()
        # 菜单回调，切换窗口可见性并调用 UIBuilder 的回调

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)
        # 时间轴事件回调，处理物理步订阅并转发给 UIBuilder

    def _on_physics_step(self, step):
        self.ui_builder.on_physics_step(step)
        # 每个物理步调用 UIBuilder 的回调

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            # 场景被打开或关闭时，进行清理
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)
        # 场景事件回调，转发给 UIBuilder

    def _build_extension_ui(self):
        # Call user function for building UI
        # 调用用户自定义的 UI 构建函数
        self.ui_builder.build_ui()
