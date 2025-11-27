#!/usr/bin/env python3
"""
测试TRAC-IK插件是否正确注册和加载
"""

import subprocess
import sys
import os

def test_plugin_files():
    """测试插件文件是否存在"""
    plugin_lib = "/home/dyk/Projects/trajectory_planning/install/trac_ik_kinematics_plugin/lib/libtrac_ik_kinematics_plugin.so"
    plugin_xml = "/home/dyk/Projects/trajectory_planning/install/trac_ik_kinematics_plugin/share/trac_ik_kinematics_plugin/trac_ik_kinematics_description.xml"

    if os.path.exists(plugin_lib):
        print("✅ 插件库文件存在:", plugin_lib)
        lib_test = True
    else:
        print("❌ 插件库文件不存在:", plugin_lib)
        lib_test = False

    if os.path.exists(plugin_xml):
        print("✅ 插件描述文件存在:", plugin_xml)
        xml_test = True
    else:
        print("❌ 插件描述文件不存在:", plugin_xml)
        xml_test = False

    return lib_test and xml_test

def test_package_discovery():
    """测试包是否被发现"""
    try:
        # 使用ros2 pkg list查看包
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode == 0:
            if "trac_ik_kinematics_plugin" in result.stdout:
                print("✅ TRAC-IK插件包被ROS2发现")
                return True
            else:
                print("❌ TRAC-IK插件包未被ROS2发现")
                return False
        else:
            print("❌ 包发现失败")
            print("错误:", result.stderr)
            return False

    except subprocess.TimeoutExpired:
        print("❌ 命令超时")
        return False
    except FileNotFoundError:
        print("❌ ros2命令未找到，请确保ROS2环境已source")
        return False

def test_package_info():
    """测试包信息"""
    try:
        # 使用ros2 pkg prefix查看包路径
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'trac_ik_kinematics_plugin'],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode == 0:
            print("✅ 插件包路径:", result.stdout.strip())
            return True
        else:
            print("❌ 无法获取插件包路径")
            print("错误:", result.stderr)
            return False

    except subprocess.TimeoutExpired:
        print("❌ 命令超时")
        return False

if __name__ == "__main__":
    print("🔍 测试TRAC-IK插件加载...")
    print()

    # 测试插件文件
    file_test = test_plugin_files()
    print()

    # 测试包发现
    pkg_test = test_package_discovery()
    print()

    # 测试包信息
    info_test = test_package_info()
    print()

    if file_test and pkg_test and info_test:
        print("🎉 所有测试通过！插件已正确编译和安装")
        print("现在可以在kinematics.yaml中使用:")
        print("  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin")
        sys.exit(0)
    else:
        print("❌ 部分测试失败，请检查:")
        print("  1. 插件是否已编译: colcon build --packages-select trac_ik_kinematics_plugin")
        print("  2. ROS2环境是否已source")
        print("  3. 工作空间是否已source: source install/setup.bash")
        sys.exit(1)