# Loading Extension
To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}
The user will see the extension appear on the toolbar on startup with the title they specified in the Extension Generator


# Extension Usage
This template extension creates a Load, Reset, and Run button in a simple UI.
The Load and Reset buttons interact with the omni.isaac.core World() in order
to simplify user interaction with the simulator and provide certain gurantees to the user
at the times their callback functions are called.  


# Template Code Overview
The template is well documented and is meant to be self-explanatory to the user should they
start reading the provided python files.  A short overview is also provided here:

global_variables.py: 
    A script that stores in global variables that the user specified when creating this extension such as the Title and Description.

extension.py:
    A class containing the standard boilerplate necessary to have the user extension show up on the Toolbar.  This
    class is meant to fulfill most ues-cases without modification.
    In extension.py, useful standard callback functions are created that the user may complete in ui_builder.py.

ui_builder.py:
    This file is the user's main entrypoint into the template.  Here, the user can see useful callback functions that have been
    set up for them, and they may also create UI buttons that are hooked up to more user-defined callback functions.  This file is
    the most thoroughly documented, and the user should read through it before making serious modification.

scenario.py:
    This file contains an implementation of an example "Scenario" that implements a "teardown", "setup", and "update" function.
    This particular structure was chosen to make a clear code separation between UI management and the scenario logic.  In this way, the 
    ExampleScenario() class serves as a simple backend to the UI.  The user should feel encouraged to implement the backend to their UI
    that best suits their needs.


# 加载扩展
要启用此扩展，请使用如下参数运行 Isaac Sim：--ext-folder {path_to_ext_folder} --enable {ext_directory_name}
用户将在启动时于工具栏看到他们在扩展生成器中指定标题的扩展。

# 扩展使用说明
此模板扩展在一个简单的 UI 中创建了“加载”、“重置”和“运行”按钮。
“加载”和“重置”按钮与 omni.isaac.core 的 World() 交互，
以简化用户与仿真器的交互，并在回调函数被调用时为用户提供一定的保证。

# 模板代码概览
该模板文档齐全，旨在让用户在阅读所提供的 python 文件时能够自解释。这里也提供一个简要概览：

global_variables.py: 
    一个脚本，用于存储用户在创建此扩展时指定的全局变量，如标题和描述。

extension.py:
    一个包含标准样板代码的类，用于让用户扩展显示在工具栏上。
    该类旨在无需修改即可满足大多数使用场景。
    在 extension.py 中，创建了一些有用的标准回调函数，用户可以在 ui_builder.py 中完善这些函数。

ui_builder.py:
    这是用户进入模板的主要入口文件。在这里，用户可以看到已为其设置的有用回调函数，
    也可以创建更多与用户自定义回调函数关联的 UI 按钮。该文件注释最为详细，用户在进行重大修改前应仔细阅读。

scenario.py:
    该文件包含了一个示例“Scenario”的实现，提供了“teardown”、“setup”和“update”函数。
    这种结构旨在清晰区分 UI 管理与场景逻辑。因此，ExampleScenario() 类作为 UI 的简单后端。
    鼓励用户根据自身需求实现最适合自己 UI 的后端逻辑。

命令：runheadless --ext-folder /workspace/work/lula_kinematics_example/ --enable Lula_Kinematics_python
