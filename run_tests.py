#!/usr/bin/env python3
"""
机器人底盘多源控制仲裁系统 - 自动化测试报告
验证优先级抢占、超时释放、拒绝响应等功能
"""
import subprocess
import time
import sys
import os
import signal
from threading import Thread, Event
from typing import List, Dict, Any

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import zmq


class TestRunner:
    """测试运行器"""
    
    def __init__(self):
        self.arbiter_process = None
        self.worker_process = None
        self.results: List[Dict[str, Any]] = []
        self.stop_event = Event()
        
    def start_arbiter(self) -> bool:
        """启动仲裁器"""
        print("[TEST] 启动仲裁器...")
        self.arbiter_process = subprocess.Popen(
            [sys.executable, 'src/chassis_arbiter.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.dirname(os.path.abspath(__file__))
        )
        time.sleep(1.0)
        if self.arbiter_process.poll() is None:
            print("[TEST] ✓ 仲裁器已启动")
            return True
        else:
            print("[TEST] ✗ 仲裁器启动失败")
            return False
    
    def start_worker(self) -> bool:
        """启动底盘执行端"""
        print("[TEST] 启动底盘执行端...")
        self.worker_process = subprocess.Popen(
            [sys.executable, 'src/chassis_worker.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=os.path.dirname(os.path.abspath(__file__))
        )
        time.sleep(0.5)
        if self.worker_process.poll() is None:
            print("[TEST] ✓ 底盘执行端已启动")
            return True
        else:
            print("[TEST] ✗ 底盘执行端启动失败")
            return False
    
    def stop_all(self):
        """停止所有进程"""
        print("\n[TEST] 清理进程...")
        if self.worker_process:
            self.worker_process.terminate()
            try:
                self.worker_process.wait(timeout=2)
            except:
                self.worker_process.kill()
        if self.arbiter_process:
            self.arbiter_process.terminate()
            try:
                self.arbiter_process.wait(timeout=2)
            except:
                self.arbiter_process.kill()
        print("[TEST] ✓ 进程已清理")
    
    def send_command(self, source: str, priority: int, vx: float, vy: float, vz: float, 
                     timeout_ms: int = 200) -> Dict[str, Any]:
        """发送控制指令并返回响应"""
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect("ipc:///tmp/chassis_arbiter.ipc")
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        
        request = {
            "source": source,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "priority": priority
        }
        
        try:
            socket.send_json(request)
            response = socket.recv_json()
            return response
        except zmq.error.Again:
            return {"success": False, "message": "timeout", "current_owner": "unknown", "current_priority": 0}
        except Exception as e:
            return {"success": False, "message": str(e), "current_owner": "unknown", "current_priority": 0}
        finally:
            socket.close()
            context.term()
    
    def add_result(self, test_name: str, passed: bool, details: str = ""):
        """记录测试结果"""
        self.results.append({
            "name": test_name,
            "passed": passed,
            "details": details
        })
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {test_name}")
        if details:
            print(f"       {details}")
    
    def test_priority_preemption(self):
        """测试优先级抢占"""
        print("\n[TEST] ========== 测试1: 优先级抢占 ==========")
        
        # web (优先级1) 先获取控制权
        r1 = self.send_command("web", 1, 0.1, 0, 0)
        self.add_result("web获取控制权", r1["success"], 
                       f"owner={r1.get('current_owner')}")
        
        # voice (优先级2) 抢占
        r2 = self.send_command("voice", 2, 0.2, 0, 0)
        expected = r2["success"] and r2.get("current_owner") == "voice"
        self.add_result("voice抢占web控制权", expected,
                       f"success={r2['success']}, owner={r2.get('current_owner')}")
        
        # auto (优先级3) 抢占
        r3 = self.send_command("auto", 3, 0.3, 0, 0)
        expected = r3["success"] and r3.get("current_owner") == "auto"
        self.add_result("auto抢占voice控制权", expected,
                       f"success={r3['success']}, owner={r3.get('current_owner')}")
        
        # emergency (优先级4) 抢占
        r4 = self.send_command("emergency", 4, 0, 0, 0)
        expected = r4["success"] and r4.get("current_owner") == "emergency"
        self.add_result("emergency抢占auto控制权", expected,
                       f"success={r4['success']}, owner={r4.get('current_owner')}")
    
    def test_priority_rejection(self):
        """测试低优先级被拒绝"""
        print("\n[TEST] ========== 测试2: 低优先级拒绝 ==========")
        
        # 先等待之前测试的控制权超时
        print("  等待之前控制权超时...")
        time.sleep(1.5)
        
        # auto (优先级3) 先获取控制权
        r1 = self.send_command("auto", 3, 0.5, 0, 0)
        self.add_result("auto获取控制权", r1["success"],
                       f"owner={r1.get('current_owner')}")
        
        # voice (优先级2) 尝试控制，应该被拒绝
        r2 = self.send_command("voice", 2, 0.2, 0, 0)
        expected = not r2["success"] and r2.get("current_owner") == "auto"
        self.add_result("voice被auto拒绝", expected,
                       f"success={r2['success']}, owner={r2.get('current_owner')}")
        
        # web (优先级1) 尝试控制，应该被拒绝
        r3 = self.send_command("web", 1, 0.1, 0, 0)
        expected = not r3["success"] and r3.get("current_owner") == "auto"
        self.add_result("web被auto拒绝", expected,
                       f"success={r3['success']}, owner={r3.get('current_owner')}")
    
    def test_timeout_release(self):
        """测试超时自动释放"""
        print("\n[TEST] ========== 测试3: 超时释放 ==========")
        
        # auto 获取控制权
        r1 = self.send_command("auto", 3, 0.3, 0, 0)
        self.add_result("auto获取控制权", r1["success"])
        
        # 等待控制权超时（1000ms + 缓冲）
        print("  等待1.5秒让控制权超时...")
        time.sleep(1.5)
        
        # web (优先级1) 应该能获取控制权
        r2 = self.send_command("web", 1, 0.1, 0, 0)
        expected = r2["success"] and r2.get("current_owner") == "web"
        self.add_result("超时后web获取控制权", expected,
                       f"success={r2['success']}, owner={r2.get('current_owner')}")
    
    def test_same_priority(self):
        """测试同优先级抢占"""
        print("\n[TEST] ========== 测试4: 同优先级抢占 ==========")
        
        # auto-A 获取控制权
        r1 = self.send_command("auto", 3, 0.3, 0, 0)
        self.add_result("auto获取控制权", r1["success"])
        
        # 另一个auto-B (同优先级3) 应该能抢占
        r2 = self.send_command("auto", 3, 0.4, 0, 0)
        expected = r2["success"]
        self.add_result("同优先级抢占", expected,
                       f"success={r2['success']}")
    
    def test_continuous_control(self):
        """测试持续控制（续期）"""
        print("\n[TEST] ========== 测试5: 持续控制续期 ==========")
        
        # 连续发送多个指令（间隔小于1秒）
        success_count = 0
        for i in range(3):
            r = self.send_command("auto", 3, 0.1 * (i+1), 0, 0)
            if r["success"]:
                success_count += 1
            time.sleep(0.3)  # 300ms间隔，小于1秒超时
        
        expected = success_count == 3
        self.add_result(f"持续控制续期 ({success_count}/3)", expected)
    
    def run_all_tests(self):
        """运行所有测试"""
        print("=" * 60)
        print("  机器人底盘多源控制仲裁系统 - 自动化测试")
        print("=" * 60)
        
        # 清理旧的IPC文件
        for f in ["/tmp/chassis_arbiter.ipc", "/tmp/chassis_final.ipc"]:
            if os.path.exists(f):
                os.remove(f)
        
        # 启动服务
        if not self.start_arbiter():
            return False
        if not self.start_worker():
            self.stop_all()
            return False
        
        # 给服务更多启动时间
        time.sleep(1.5)
        
        try:
            # 运行测试
            self.test_priority_preemption()
            time.sleep(0.5)
            
            self.test_priority_rejection()
            time.sleep(0.5)
            
            self.test_timeout_release()
            time.sleep(0.5)
            
            self.test_same_priority()
            time.sleep(0.5)
            
            self.test_continuous_control()
            
        except Exception as e:
            print(f"\n[TEST] 测试异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop_all()
        
        # 输出测试报告
        self.print_report()
        return all(r["passed"] for r in self.results)
    
    def print_report(self):
        """打印测试报告"""
        print("\n" + "=" * 60)
        print("  测试报告")
        print("=" * 60)
        
        passed = sum(1 for r in self.results if r["passed"])
        failed = sum(1 for r in self.results if not r["passed"])
        total = len(self.results)
        
        print(f"\n总测试数: {total}")
        print(f"通过: {passed} ✓")
        print(f"失败: {failed} ✗")
        print(f"通过率: {passed/total*100:.1f}%")
        
        if failed > 0:
            print("\n失败的测试:")
            for r in self.results:
                if not r["passed"]:
                    print(f"  ✗ {r['name']}")
                    if r['details']:
                        print(f"    {r['details']}")
        
        print("\n" + "=" * 60)
        if failed == 0:
            print("  🎉 所有测试通过！")
        else:
            print("  ⚠️ 部分测试失败，请检查实现")
        print("=" * 60)


def main():
    runner = TestRunner()
    success = runner.run_all_tests()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
