#!/usr/bin/env python3
"""
HomeBot 机械臂遥操作 GUI 打包脚本

使用 PyInstaller 将 applications/arm_teleop/gui.py 单独打包为 exe。
针对 Anaconda/venv 环境下 tkinter DLL 和数据文件不在 sys.prefix 的情况做了自动收集。
"""
import glob
import os
import subprocess
import sys


# PyInstaller 在 Anaconda 环境下常常解析不到 Library/bin 中的运行时 DLL，
# 导致 _ctypes / _ssl 等 C 扩展无法加载。这里显式补齐最常见的几个。
_MISSING_CONDA_DLLS = [
    "ffi.dll",
    "libcrypto-3-x64.dll",
    "libssl-3-x64.dll",
    "libexpat.dll",
    "liblzma.dll",
    "libbz2.dll",
    "libmpdec-4.dll",
]


def _add_conda_runtime_dlls(base_dir: str, cmd: list) -> None:
    bin_dir = os.path.join(base_dir, "Library", "bin")
    if not os.path.isdir(bin_dir):
        return
    for name in _MISSING_CONDA_DLLS:
        dll_path = os.path.join(bin_dir, name)
        if os.path.exists(dll_path):
            cmd.extend(["--add-binary", f"{dll_path};."])
            print(f"添加运行时 DLL: {dll_path}")


def _find_tcltk(base_dir: str) -> tuple:
    """
    在 base_dir 下查找 Tcl/Tk 的 DLL、tcl lib 目录和 tk lib 目录。

    Returns:
        (tcl_dll, tk_dll, tcl_lib_dir, tk_lib_dir) 或 (None, None, None, None)
    """
    bin_dir = os.path.join(base_dir, "Library", "bin")
    lib_dir = os.path.join(base_dir, "Library", "lib")

    tcl_dlls = sorted(glob.glob(os.path.join(bin_dir, "tcl*.dll")))
    tk_dlls = sorted(glob.glob(os.path.join(bin_dir, "tk*.dll")))

    # 优先使用带小版本号的目录，例如 tcl8.6 / tk8.6
    tcl_lib = os.path.join(lib_dir, "tcl8.6")
    tk_lib = os.path.join(lib_dir, "tk8.6")
    if not os.path.isdir(tcl_lib):
        candidates = sorted(glob.glob(os.path.join(lib_dir, "tcl8*")))
        tcl_lib = candidates[0] if candidates else None
    if not os.path.isdir(tk_lib):
        candidates = sorted(glob.glob(os.path.join(lib_dir, "tk8*")))
        tk_lib = candidates[0] if candidates else None

    return (
        tcl_dlls[0] if tcl_dlls else None,
        tk_dlls[0] if tk_dlls else None,
        tcl_lib,
        tk_lib,
    )


def main() -> None:
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    src_dir = os.path.join(project_root, "src")
    entry = os.path.join(src_dir, "applications", "arm_teleop", "gui.py")

    if not os.path.exists(entry):
        print(f"入口文件不存在: {entry}")
        sys.exit(1)

    # 定位 base Python（venv 中的 tkinter 实际来自 base interpreter）
    try:
        import _tkinter
        base_python_dir = os.path.dirname(os.path.dirname(_tkinter.__file__))
    except Exception as e:
        print(f"无法定位 _tkinter: {e}")
        base_python_dir = None

    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--noconfirm",
        "--onefile",
        "--windowed",
        "--name", "HomeBotArmTeleop",
        "--paths", src_dir,
        "--hidden-import", "zmq",
        "--hidden-import", "zmq.backend.cython",
        "--hidden-import", "serial.tools.list_ports",
        "--hidden-import", "hal.scservo_sdk",
        "--collect-submodules", "hal.scservo_sdk",
        "--collect-data", "hal.scservo_sdk",
        # 排除 HomeBot 其他应用使用的大依赖，减小体积
        "--exclude-module", "opencv_python",
        "--exclude-module", "cv2",
        "--exclude-module", "flask",
        "--exclude-module", "flask_socketio",
        "--exclude-module", "ultralytics",
        "--exclude-module", "numpy",
        "--exclude-module", "filterpy",
        "--exclude-module", "sherpa_onnx",
        "--exclude-module", "sounddevice",
        "--exclude-module", "openai",
        "--exclude-module", "fastmcp",
        "--exclude-module", "websockets",
        "--exclude-module", "volcengine",
        "--exclude-module", "serial.tools.list_ports_osx",
        "--exclude-module", "serial.tools.list_ports_linux",
    ]

    if base_python_dir:
        _add_conda_runtime_dlls(base_python_dir, cmd)
        tcl_dll, tk_dll, tcl_lib, tk_lib = _find_tcltk(base_python_dir)
        if tcl_dll and os.path.exists(tcl_dll):
            cmd.extend(["--add-binary", f"{tcl_dll};."])
            print(f"添加 Tcl DLL: {tcl_dll}")
        if tk_dll and os.path.exists(tk_dll):
            cmd.extend(["--add-binary", f"{tk_dll};."])
            print(f"添加 Tk DLL: {tk_dll}")
        if tcl_lib and os.path.isdir(tcl_lib):
            # 目标路径保持为 lib/tcl8.x，与运行时 sys._MEIPASS + '/lib/tcl8.x' 对应
            lib_name = os.path.basename(tcl_lib)
            cmd.extend(["--add-data", f"{tcl_lib};lib/{lib_name}"])
            print(f"添加 Tcl lib: {tcl_lib} -> lib/{lib_name}")
        if tk_lib and os.path.isdir(tk_lib):
            lib_name = os.path.basename(tk_lib)
            cmd.extend(["--add-data", f"{tk_lib};lib/{lib_name}"])
            print(f"添加 Tk lib: {tk_lib} -> lib/{lib_name}")

    cmd.append(entry)

    print("执行命令:")
    print(" ".join(cmd))
    print()

    try:
        subprocess.check_call(cmd)
        print("\n打包完成，输出: dist/HomeBotArmTeleop.exe")
    except subprocess.CalledProcessError as e:
        print(f"\n打包失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
