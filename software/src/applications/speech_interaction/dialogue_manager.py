"""对话管理器 - 处理用户语音输入并调用工具

支持 LLM API 和 MCP 工具调用
"""
import json
import asyncio
from typing import AsyncGenerator

from common.logging import get_logger
from configs.config import get_config
from configs.secrets import require_secrets
from openai import OpenAI

logger = get_logger(__name__)


class DialogueManager:
    """对话管理器类"""
    
    def __init__(self):
        """初始化对话管理器"""
        # 确保密钥已配置
        require_secrets("llm")
        
        llm_config = get_config().llm
        self.context = {
            "history": []
        }
        self.system_prompt = self._get_system_prompt()
        
        # 初始化 OpenAI 客户端
        self.client = OpenAI(
            api_key=llm_config.api_key,
            base_url=llm_config.api_url
        )
        self.model = llm_config.model
        self.temperature = llm_config.temperature
        self.max_tokens = llm_config.max_tokens
        self.top_p = getattr(llm_config, 'top_p', 0.9)
        
        # MCP 相关 - 延迟初始化：仅在第一次对话时才初始化 MCP 客户端
        self.mcp_client = None
        self.mcp_tools = []
        self._mcp_initialized = False  # 标记 MCP 是否已初始化，避免启动时自动初始化
    
    def _get_system_prompt(self) -> str:
        """获取系统提示词
        
        Returns:
            str: 系统提示词
        """
        return """你是HomeBot，一个智能语音控制的家庭机器人助手。

你可以控制机器人的移动底盘和机械臂。当前机械臂控制功能完全可用，请积极调用工具执行用户的机械臂控制指令。

【底盘控制】
- move_forward(distance, speed): 向前移动指定距离（米）
- move_backward(distance, speed): 向后移动指定距离（米）
- turn_left(angle, speed): 向左旋转指定角度（度）
- turn_right(angle, speed): 向右旋转指定角度（度）
- stop_robot(): 立即停止机器人
- get_robot_status(): 获取机器人状态
- get_battery_status(): 获取电池电量和电压状态

【机械臂控制 - 功能已就绪】
- reset_arm(): 机械臂复位，恢复到初始姿态（休息位置）
- raise_arm(distance): 抬高机械臂末端（Z方向向上），distance: 距离（米）0.01-0.1，默认 0.03
- lower_arm(distance): 放低机械臂末端（Z方向向下），distance: 距离（米）0.01-0.1，默认 0.03
- extend_arm(distance): 前伸机械臂末端（X方向向前），distance: 距离（米）0.01-0.1，默认 0.03
- retract_arm(distance): 后退机械臂末端（X方向向后），distance: 距离（米）0.01-0.1，默认 0.03
- rotate_arm_left(angle): 机械臂基座左转，angle: 角度 5-180，默认 30
- rotate_arm_right(angle): 机械臂基座右转，angle: 角度 5-180，默认 30
- grab_object(): 执行抓取动作/关闭夹爪
- release_object(): 执行释放/松开动作/打开夹爪
- hold_object(): "帮我拿着这个" - 复合动作：复位→打开夹爪→等待2秒→关闭夹爪
- move_arm_to_position(joint_angles): 移动机械臂到指定关节角度

重要规则：
1. 用户要求控制机械臂或底盘时，必须强制调用工具，不要只回复文字不干活
2. 调用工具完成后根据执行结果简洁告知用户（如"已完成"、"机械臂已抬高"）
3. 闲聊问答直接回复，回复要简洁（20字以内）
4. 绝对禁止在回复中出现move_、turn_、grab_等工具函数名称

注意事项：
- 用户语音可能有错别字，如"机械臂复位"识别成"机械臂服务"，请自主理解

回复示例：
- 用户说"机械臂抬高一点" → 你回复"好的"
- 工具执行完成后 → 你回复"机械臂已抬高"

"""

    async def _initialize_mcp_client(self):
        """初始化 MCP 客户端
        
        可在应用启动时调用，提前初始化 MCP，避免第一次对话时才初始化导致的延迟。
        重复调用时如果已初始化则跳过。
        """
        if self._mcp_initialized:
            logger.debug("MCP 客户端已初始化，跳过")
            return
            
        try:
            from applications.speech_interaction.mcp_server import get_mcp_client
            self.mcp_client = get_mcp_client()
            self.mcp_tools = self.mcp_client.tools
            self._mcp_initialized = True
            logger.info(f"MCP 客户端初始化成功，可用工具: {[t['function']['name'] for t in self.mcp_tools]}")
        except Exception as e:
            logger.error(f"MCP 客户端初始化失败: {e}")
            self.mcp_client = None
            self.mcp_tools = []
    
    async def _get_mcp_tools(self) -> list:
        """获取 MCP 工具列表
        
        Returns:
            list: 工具列表
        """
        return []
    
    async def _call_mcp_tool(self, tool_name: str, arguments: dict) -> dict:
        """调用 MCP 工具
        
        Args:
            tool_name: 工具名称
            arguments: 工具参数
            
        Returns:
            dict: 工具调用结果
        """
        if not self.mcp_client:
            logger.error("MCP 客户端未初始化")
            return {"status": "error", "message": "MCP 客户端未初始化"}
        
        try:
            result = await self.mcp_client.call_tool(tool_name, arguments)
            return result
        except Exception as e:
            logger.error(f"调用 MCP 工具失败: {e}")
            return {"status": "error", "message": str(e)}
    
    def _call_llm_api(self, messages: list) -> dict:
        """调用 LLM API
        
        Args:
            messages: 对话历史消息列表
        
        Returns:
            dict: LLM API 返回结果
        """
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                top_p=self.top_p,
                tools=self.mcp_tools if self.mcp_tools else None,
                tool_choice="auto" if self.mcp_tools else None,
                extra_body={"thinking": {"type": "disabled"}}  # 显式禁用思考功能，提升响应速度
            )
            
            # 转换为统一格式
            response_dict = {
                "choices": [
                    {
                        "message": {
                            "content": response.choices[0].message.content,
                            "tool_calls": []
                        }
                    }
                ]
            }
            
            # 添加工具调用信息
            if response.choices[0].message.tool_calls:
                for tool_call in response.choices[0].message.tool_calls:
                    response_dict["choices"][0]["message"]["tool_calls"].append({
                        "id": tool_call.id,
                        "type": tool_call.type,
                        "function": {
                            "name": tool_call.function.name,
                            "arguments": tool_call.function.arguments
                        }
                    })
            
            return response_dict
        except Exception as e:
            logger.error(f"LLM API调用异常: {e}")
            return {}
    
    async def process_query(self, text: str, context: dict = None) -> AsyncGenerator[tuple[str, dict], None]:
        """处理用户查询
        
        Args:
            text: 用户输入文本
            context: 对话上下文（可选）
        
        Yields:
            tuple: (回复文本, 更新后的上下文)
        """
        if not text:
            yield "抱歉，我没听清，请再说一遍", self.context
            return
        
        # 确保 MCP 客户端已初始化
        if not self._mcp_initialized:
            await self._initialize_mcp_client()
        
        current_context = context or self.context
        
        try:
            # 构建对话历史
            messages = [
                {"role": "system", "content": self.system_prompt}
            ]
            
            # 添加历史对话
            for msg in current_context["history"]:
                messages.append(msg)
            
            # 添加当前用户输入
            messages.append({"role": "user", "content": text})
            
            # 调用 LLM API
            response = self._call_llm_api(messages)
            
            if not response or "choices" not in response:
                yield "抱歉，我没听清，请再说一遍", current_context
                return
            
            llm_message = response["choices"][0]["message"]
            logger.info(f"LLM回复: {llm_message.get('content', '无内容')}")

            # 处理工具调用
            if llm_message.get("tool_calls"):
                logger.info(f"检测到工具调用: {llm_message['tool_calls']}")
                
                # 第一次调用LLM后，如果LLM已经说了"好的"等内容，直接使用
                # 否则给一个默认的"好的"响应
                initial_reply = llm_message.get("content", "")
                if initial_reply and initial_reply.strip() and initial_reply.strip() != "None":
                    # LLM已经生成了回复（如"好的"），先播报给用户
                    yield initial_reply, current_context
                else:
                    # LLM没有生成回复内容，我们给一个默认的"好的"
                    yield "好的", current_context
                
                # 依次执行所有工具调用
                tool_results = []
                for tool_call in llm_message["tool_calls"]:
                    tool_name = tool_call["function"]["name"]
                    tool_args = json.loads(tool_call["function"]["arguments"])
                    
                    logger.info(f"执行工具调用: {tool_name}, 参数: {tool_args}")
                    
                    # 静默调用 MCP 工具（不再播报"正在执行xxx"）
                    tool_result = await self._call_mcp_tool(tool_name, tool_args)
                    logger.info(f"工具调用结果: {tool_result}")
                    
                    # 将工具调用结果添加到对话历史
                    messages.append({
                        "role": "assistant",
                        "content": None,
                        "tool_calls": [tool_call]
                    })
                    
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call["id"],
                        "name": tool_name,
                        "content": json.dumps(tool_result)
                    })
                    
                    tool_results.append(tool_result)

                # 构建工具执行结果的汇总信息
                result_messages = []
                for result in tool_results:
                    msg = result.get('message', '执行完成')
                    result_messages.append(msg)
                combined_result = "；".join(result_messages)

                # 添加用户提示，让 LLM 根据结果生成回复（要求不要出现工具函数名）
                messages.append({
                    "role": "user", 
                    "content": f"工具调用已完成，结果: {combined_result}。请简洁地告知用户执行结果，绝对不要提及工具函数名称。"
                })
                
                # 再次调用 LLM，获取最终回复
                response = self._call_llm_api(messages)
                llm_message = response["choices"][0]["message"]
                reply = llm_message.get("content", "执行完成")
            else:
                # 没有工具调用，直接使用 LLM 回复
                reply = llm_message.get("content", "抱歉，我没听清，请再说一遍")
            
            # 更新对话历史
            current_context["history"].append({"role": "user", "content": text})
            current_context["history"].append({"role": "assistant", "content": reply})
            
            # 限制历史记录长度
            if len(current_context["history"]) > 20:
                current_context["history"] = current_context["history"][-20:]

            self.context = current_context
            
            logger.info(f"最终回复: {reply}")
            
            yield reply, current_context
            
        except Exception as e:
            logger.error(f"处理用户查询失败: {e}")
            yield "抱歉，处理出错了，请再试一次", current_context
            yield "抱歉，我现在有点忙，请稍后再试", current_context
    
    def clear_context(self):
        """清除对话上下文"""
        self.context = {"history": []}
        logger.info("对话上下文已清除")
    
    def get_context(self) -> dict:
        """获取当前对话上下文
        
        Returns:
            dict: 当前对话上下文
        """
        return self.context
