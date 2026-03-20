"""对话管理器 - 处理用户语音输入并调用工具

支持 LLM API 和 MCP 工具调用
"""
import json
import asyncio
from typing import AsyncGenerator

from common.logging import get_logger
from configs.config import get_config
from openai import OpenAI

logger = get_logger(__name__)


class DialogueManager:
    """对话管理器类"""
    
    def __init__(self):
        """初始化对话管理器"""
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
        
        # MCP 相关
        self.mcp_client = None
        self.mcp_tools = []
        self._mcp_initialized = False
    
    def _get_system_prompt(self) -> str:
        """获取系统提示词
        
        Returns:
            str: 系统提示词
        """
        return """你是HomeBot，一个智能语音控制的家庭机器人助手。

你可以控制机器人的移动底盘，支持以下动作：
- move_forward(distance, speed): 向前移动指定距离（米）
- move_backward(distance, speed): 向后移动指定距离（米）
- turn_left(angle, speed): 向左旋转指定角度（度）
- turn_right(angle, speed): 向右旋转指定角度（度）
- stop_robot(): 立即停止机器人
- get_robot_status(): 获取机器人状态

重要规则：
1. 当用户要求移动或转向时，必须调用相应工具，不要只是回复文字
2. 工具调用后会返回执行结果，请根据结果告知用户
3. 如果用户说"向前走一米"，调用 move_forward(distance=1.0)
4. 如果用户说"左转90度"，调用 turn_left(angle=90)
5. 对于闲聊、问答，直接回复即可
6. 回复要简洁（20字以内），适合语音播报

当前机器人底盘已连接，可以正常控制。
"""
    
    async def _initialize_mcp_client(self):
        """初始化 MCP 客户端"""
        if self._mcp_initialized:
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
                tools=self.mcp_tools if self.mcp_tools else None,
                tool_choice="auto" if self.mcp_tools else None
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
                
                for tool_call in llm_message["tool_calls"]:
                    tool_name = tool_call["function"]["name"]
                    tool_args = json.loads(tool_call["function"]["arguments"])
                    
                    logger.info(f"执行工具调用: {tool_name}, 参数: {tool_args}")
                    
                    # 先返回一个"正在执行"的消息
                    yield f"好的，正在执行{tool_name}", current_context
                    
                    # 调用 MCP 工具
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

                # 添加用户提示，让 LLM 根据结果生成回复
                messages.append({
                    "role": "user", 
                    "content": f"工具调用已完成，结果: {tool_result.get('message', '执行完成')}。请简洁地告知用户执行结果。"
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
