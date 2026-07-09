#!/usr/bin/env python3
"""
HomeBot 机械臂 WLAN 遥操作 —— 简易 Tkinter GUI

支持：
- 连接/启动应用
- 开启/关闭遥操作
- 录制动作并保存
- 回放动作（可调速度、循环）
- 实时日志显示

可作为 PyInstaller 打包入口。
"""
import glob
import logging
import os
import queue
import sys
import threading

try:
    from serial.tools.list_ports import comports
except Exception:  # pragma: no cover
    comports = None  # 没有 pyserial 时仍可手动输入

# PyInstaller 打包后，若 Tcl/Tk 库目录不在默认位置，需手动指定
if getattr(sys, "frozen", False):
    _meipass = getattr(sys, "_MEIPASS", None)
    if _meipass:
        _tcl_libs = glob.glob(os.path.join(_meipass, "lib", "tcl8*"))
        _tk_libs = glob.glob(os.path.join(_meipass, "lib", "tk8*"))
        if _tcl_libs:
            os.environ.setdefault("TCL_LIBRARY", _tcl_libs[0])
        if _tk_libs:
            os.environ.setdefault("TK_LIBRARY", _tk_libs[0])

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# 添加项目根目录到路径（用于直接运行和 PyInstaller）
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_SRC = os.path.normpath(os.path.join(_SCRIPT_DIR, "../.."))
if _PROJECT_SRC not in sys.path:
    sys.path.insert(0, _PROJECT_SRC)

from configs import get_config
from applications.arm_teleop.app import ArmTeleopApp, build_master_arm_config, TeleopMode


class GuiLogHandler(logging.Handler):
    """把日志记录放入队列，供 GUI 主线程读取显示"""

    def __init__(self, log_queue: queue.Queue):
        super().__init__()
        self.log_queue = log_queue
        self.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.log_queue.put(self.format(record))
        except Exception:
            self.handleError(record)


class ArmTeleopGUI:
    """机械臂遥操作图形界面"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("HomeBot 机械臂遥操作")
        self.root.geometry("700x750")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        self.app: ArmTeleopApp | None = None
        self.app_thread: threading.Thread | None = None
        self.log_queue: queue.Queue = queue.Queue()
        self._stop_polling = False

        self._build_ui()
        self._setup_logging()
        self._poll_log()
        self._poll_status()

    # ==================== UI 构建 ====================

    def _build_ui(self) -> None:
        cfg = get_config()
        padx = 10
        pady = 5

        # 连接参数
        conn_frame = ttk.LabelFrame(self.root, text="连接参数", padding=10)
        conn_frame.pack(fill=tk.X, padx=padx, pady=pady)

        ttk.Label(conn_frame, text="从臂地址:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=pady)
        self.slave_addr_var = tk.StringVar(value=cfg.arm_teleop.slave_arm_addr)
        ttk.Entry(conn_frame, textvariable=self.slave_addr_var, width=45).grid(row=0, column=1, padx=5, pady=pady)

        ttk.Label(conn_frame, text="主臂串口:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=pady)
        self.port_var = tk.StringVar(value=cfg.arm.serial_port)
        self.port_combo = ttk.Combobox(
            conn_frame,
            textvariable=self.port_var,
            width=40,
            state="normal",
            postcommand=self._refresh_ports,
        )
        self.port_combo.grid(row=1, column=1, padx=5, pady=pady)
        ttk.Button(conn_frame, text="刷新", command=self._refresh_ports, width=6).grid(
            row=1, column=2, padx=5, pady=pady
        )
        self._refresh_ports()

        self.torque_off_var = tk.BooleanVar(value=cfg.arm_teleop.torque_off)
        ttk.Checkbutton(
            conn_frame,
            text="关闭主臂扭矩（方便手动拖动）",
            variable=self.torque_off_var,
        ).grid(row=2, column=0, columnspan=2, sticky=tk.W, padx=5, pady=pady)

        self.start_btn = ttk.Button(conn_frame, text="连接 / 启动", command=self._start_app)
        self.start_btn.grid(row=3, column=0, padx=5, pady=pady)

        self.stop_btn = ttk.Button(conn_frame, text="断开 / 停止", command=self._stop_app, state=tk.DISABLED)
        self.stop_btn.grid(row=3, column=1, sticky=tk.W, padx=5, pady=pady)

        # 遥操作控制
        teleop_frame = ttk.LabelFrame(self.root, text="遥操作", padding=10)
        teleop_frame.pack(fill=tk.X, padx=padx, pady=pady)

        self.enable_btn = ttk.Button(teleop_frame, text="开启遥操作", command=self._enable_teleop, state=tk.DISABLED)
        self.enable_btn.pack(side=tk.LEFT, padx=5)

        self.disable_btn = ttk.Button(teleop_frame, text="关闭遥操作", command=self._disable_teleop, state=tk.DISABLED)
        self.disable_btn.pack(side=tk.LEFT, padx=5)

        # 录制
        record_frame = ttk.LabelFrame(self.root, text="录制", padding=10)
        record_frame.pack(fill=tk.X, padx=padx, pady=pady)

        rec_path = os.path.join(cfg.arm_teleop.trajectory_dir, "record.json")
        self.record_path_var = tk.StringVar(value=rec_path)
        ttk.Entry(record_frame, textvariable=self.record_path_var, width=50).grid(row=0, column=0, padx=5, pady=pady)
        ttk.Button(record_frame, text="浏览", command=self._browse_record_file).grid(row=0, column=1, padx=5, pady=pady)

        self.record_start_btn = ttk.Button(record_frame, text="开始录制", command=self._start_recording, state=tk.DISABLED)
        self.record_start_btn.grid(row=1, column=0, sticky=tk.W, padx=5, pady=pady)

        self.record_stop_btn = ttk.Button(record_frame, text="停止录制", command=self._stop_recording, state=tk.DISABLED)
        self.record_stop_btn.grid(row=1, column=1, sticky=tk.W, padx=5, pady=pady)

        # 回放
        playback_frame = ttk.LabelFrame(self.root, text="回放", padding=10)
        playback_frame.pack(fill=tk.X, padx=padx, pady=pady)

        self.playback_path_var = tk.StringVar(value=cfg.arm_teleop.default_playback_file)
        ttk.Entry(playback_frame, textvariable=self.playback_path_var, width=50).grid(row=0, column=0, padx=5, pady=pady)
        ttk.Button(playback_frame, text="浏览", command=self._browse_playback_file).grid(row=0, column=1, padx=5, pady=pady)

        ttk.Label(playback_frame, text="速度:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=pady)
        self.playback_speed_var = tk.StringVar(value="1.0")
        ttk.Entry(playback_frame, textvariable=self.playback_speed_var, width=8).grid(row=1, column=0, sticky=tk.W, padx=(60, 0), pady=pady)

        ttk.Label(playback_frame, text="循环:").grid(row=1, column=0, sticky=tk.W, padx=(140, 0), pady=pady)
        self.playback_loop_var = tk.StringVar(value="1")
        ttk.Entry(playback_frame, textvariable=self.playback_loop_var, width=6).grid(row=1, column=0, sticky=tk.W, padx=(190, 0), pady=pady)

        self.loop_forever_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(playback_frame, text="无限循环", variable=self.loop_forever_var).grid(row=1, column=0, sticky=tk.W, padx=(260, 0), pady=pady)

        self.playback_start_btn = ttk.Button(playback_frame, text="开始回放", command=self._start_playback, state=tk.DISABLED)
        self.playback_start_btn.grid(row=2, column=0, sticky=tk.W, padx=5, pady=pady)

        self.playback_stop_btn = ttk.Button(playback_frame, text="停止回放", command=self._stop_playback, state=tk.DISABLED)
        self.playback_stop_btn.grid(row=2, column=1, sticky=tk.W, padx=5, pady=pady)

        # 状态栏
        self.status_var = tk.StringVar(value="状态: 未启动")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(fill=tk.X, padx=padx, pady=2)

        # 日志区
        log_frame = ttk.LabelFrame(self.root, text="日志", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=padx, pady=pady)

        self.log_text = tk.Text(log_frame, wrap=tk.WORD, state=tk.DISABLED, height=15)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)

    def _detect_ports(self) -> list[str]:
        """检测可用串口，返回 ['COM3 - description', ...] 形式列表"""
        if comports is None:
            return []
        try:
            return [f"{p.device} - {p.description}" for p in comports()]
        except Exception as e:
            logging.warning(f"串口检测失败: {e}")
            return []

    def _refresh_ports(self) -> None:
        """刷新串口下拉列表"""
        ports = self._detect_ports()
        self.port_combo["values"] = ports

        current = self.port_var.get().strip()
        current_device = current.split(" - ")[0] if current else ""

        selected = None
        for p in ports:
            if p.startswith(current_device):
                selected = p
                break

        if selected:
            self.port_var.set(selected)
        elif ports:
            self.port_var.set(ports[0])

    def _get_selected_port(self) -> str:
        """从下拉框当前值中提取串口设备名"""
        return self.port_var.get().strip().split(" - ")[0]

    # ==================== 日志与状态轮询 ====================

    def _setup_logging(self) -> None:
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.INFO)

        handler = GuiLogHandler(self.log_queue)
        handler.setLevel(logging.INFO)
        root_logger.addHandler(handler)

        # 同时写入日志文件，便于打包后排查问题
        try:
            if getattr(sys, "frozen", False):
                base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
                log_dir = os.path.join(base, "HomeBot", "logs")
            else:
                log_dir = os.path.join(_PROJECT_SRC, "logs")
            os.makedirs(log_dir, exist_ok=True)
            file_handler = logging.FileHandler(
                os.path.join(log_dir, "arm_teleop.log"), encoding="utf-8", mode="a"
            )
            file_handler.setFormatter(
                logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
            )
            file_handler.setLevel(logging.DEBUG)
            root_logger.addHandler(file_handler)
        except Exception as e:
            logging.warning(f"无法创建文件日志处理器: {e}")

    def _poll_log(self) -> None:
        if self._stop_polling:
            return
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self._append_log(msg)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_log)

    def _append_log(self, msg: str) -> None:
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def _poll_status(self) -> None:
        if self._stop_polling:
            return
        self._update_ui_state()
        self.root.after(200, self._poll_status)

    # ==================== 应用控制 ====================

    def _start_app(self) -> None:
        if self.app is not None:
            return

        cfg = get_config()
        teleop_cfg = cfg.arm_teleop
        teleop_cfg.slave_arm_addr = self.slave_addr_var.get().strip()
        teleop_cfg.torque_off = self.torque_off_var.get()

        arm_cfg = cfg.arm
        arm_cfg.serial_port = self._get_selected_port()

        master_cfg = build_master_arm_config(arm_cfg)
        self.app = ArmTeleopApp(teleop_cfg, master_cfg)

        self.app_thread = threading.Thread(target=self._run_app, daemon=True)
        self.app_thread.start()

        self._set_connected_ui(True)

    def _run_app(self) -> None:
        try:
            if not self.app.initialize():
                self._append_log("应用初始化失败")
                self._set_connected_ui(False)
                return
            self.app.run()
        except Exception as e:
            logging.exception(f"应用运行异常: {e}")
        finally:
            self.app = None
            self._set_connected_ui(False)

    def _stop_app(self) -> None:
        if self.app:
            self.app.stop()
        if self.app_thread:
            self.app_thread.join(timeout=2.0)
        self.app = None
        self.app_thread = None
        self._set_connected_ui(False)

    def _enable_teleop(self) -> None:
        if self.app:
            self.app.set_enabled(True)

    def _disable_teleop(self) -> None:
        if self.app:
            self.app.set_enabled(False)

    def _start_recording(self) -> None:
        if not self.app:
            return
        self.app._record_file = self.record_path_var.get().strip()
        self.app._start_recording()

    def _stop_recording(self) -> None:
        if self.app:
            self.app._stop_recording()

    def _start_playback(self) -> None:
        if not self.app:
            return
        path = self.playback_path_var.get().strip()
        if not path:
            messagebox.showwarning("提示", "请选择回放文件")
            return
        try:
            speed = float(self.playback_speed_var.get())
        except ValueError:
            messagebox.showwarning("提示", "回放速度必须是数字")
            return
        if self.loop_forever_var.get():
            loop = 0
        else:
            try:
                loop = int(self.playback_loop_var.get())
                if loop < 1:
                    loop = 1
            except ValueError:
                loop = 1
        self.app.start_playback(path, speed, loop)

    def _stop_playback(self) -> None:
        if self.app:
            self.app._stop_playback()

    # ==================== UI 辅助 ====================

    def _browse_record_file(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=get_config().arm_teleop.trajectory_dir,
        )
        if path:
            self.record_path_var.set(path)

    def _browse_playback_file(self) -> None:
        path = filedialog.askopenfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=get_config().arm_teleop.trajectory_dir,
        )
        if path:
            self.playback_path_var.set(path)

    def _set_connected_ui(self, connected: bool) -> None:
        """从后台线程调用时，通过 after 切到主线程更新 UI"""
        self.root.after(0, lambda: self._update_connection_buttons(connected))

    def _update_connection_buttons(self, connected: bool) -> None:
        if connected:
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
        else:
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            self._update_control_buttons(False, None)

    def _update_ui_state(self) -> None:
        if self.app is None:
            self.status_var.set("状态: 未启动")
            self._update_control_buttons(False, None)
            return

        mode = self.app.mode
        enabled = self.app.enabled
        mode_text = {TeleopMode.TELEOP: "遥操作", TeleopMode.RECORDING: "录制中", TeleopMode.PLAYBACK: "回放中"}.get(
            mode, "未知"
        )
        self.status_var.set(f"状态: {mode_text} | 遥操作: {'开' if enabled else '关'}")
        self._update_control_buttons(True, mode)

    def _update_control_buttons(self, connected: bool, mode) -> None:
        if not connected:
            self.enable_btn.config(state=tk.DISABLED)
            self.disable_btn.config(state=tk.DISABLED)
            self.record_start_btn.config(state=tk.DISABLED)
            self.record_stop_btn.config(state=tk.DISABLED)
            self.playback_start_btn.config(state=tk.DISABLED)
            self.playback_stop_btn.config(state=tk.DISABLED)
            return

        is_playback = mode == TeleopMode.PLAYBACK
        is_recording = mode == TeleopMode.RECORDING

        self.enable_btn.config(state=tk.DISABLED if self.app.enabled or is_playback else tk.NORMAL)
        self.disable_btn.config(state=tk.NORMAL if self.app.enabled and not is_playback else tk.DISABLED)

        self.record_start_btn.config(state=tk.DISABLED if is_recording or is_playback else tk.NORMAL)
        self.record_stop_btn.config(state=tk.NORMAL if is_recording else tk.DISABLED)

        self.playback_start_btn.config(state=tk.DISABLED if is_playback or is_recording else tk.NORMAL)
        self.playback_stop_btn.config(state=tk.NORMAL if is_playback else tk.DISABLED)

    def _on_closing(self) -> None:
        self._stop_polling = True
        self._stop_app()
        self.root.destroy()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="HomeBot 机械臂遥操作 GUI")
    parser.add_argument("--auto-start", action="store_true", help="启动后自动连接主臂")
    args, _ = parser.parse_known_args()

    root = tk.Tk()
    app = ArmTeleopGUI(root)
    if args.auto_start:
        root.after(500, app._start_app)
    root.mainloop()


if __name__ == "__main__":
    main()
