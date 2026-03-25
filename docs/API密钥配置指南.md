# HomeBot API 密钥配置指南

本文档说明如何安全地管理 HomeBot 项目中的各类 API 密钥。

## 🔐 支持的密钥类型

| 服务 | 环境变量名 | 说明 |
|------|-----------|------|
| **火山引擎 TTS** | `VOLCANO_APPID`<br>`VOLCANO_ACCESS_TOKEN` | 语音合成服务 |
| **DeepSeek LLM** | `DEEPSEEK_API_KEY` | 大语言模型对话 |
| **图片理解** | `VISION_API_KEY` | 视觉分析（可选，默认同 LLM） |

## 📁 配置文件

### 推荐方式：`.env.local` 文件

在项目 `software/` 目录下创建 `.env.local` 文件：

```bash
cd software
cp .env.example .env.local
# 编辑 .env.local 填入你的密钥
```

### 文件格式

```ini
# 火山引擎 TTS 配置
VOLCANO_APPID=your_appid_here
VOLCANO_ACCESS_TOKEN=your_token_here

# DeepSeek LLM 配置
DEEPSEEK_API_KEY=sk-your_api_key_here

# 图片理解配置（可选，留空则使用 DeepSeek）
# VISION_API_KEY=your_vision_api_key
```

## 🚀 快速配置

### 方式一：使用配置向导（推荐）

```bash
cd software
python tools/setup_secrets.py
```

按提示输入密钥即可自动创建 `.env.local` 文件。

### 方式二：手动创建

1. 复制模板文件：
   ```bash
   cp software/.env.example software/.env.local
   ```

2. 编辑 `.env.local`，填入从各平台获取的密钥

3. 验证配置：
   ```bash
   python tools/check_config.py
   ```

### 方式三：系统环境变量

在系统环境变量中设置（适合 CI/CD 或 Docker）：

```bash
# Linux/macOS
export VOLCANO_APPID=your_appid
export VOLCANO_ACCESS_TOKEN=your_token
export DEEPSEEK_API_KEY=sk-your_key

# Windows PowerShell
$env:VOLCANO_APPID="your_appid"
$env:DEEPSEEK_API_KEY="sk-your_key"
```

## 🔍 密钥获取指南

### 火山引擎 TTS

1. 访问 [火山引擎控制台](https://console.volcengine.com/)
2. 进入「语音识别」->「语音技术」
3. 创建应用获取 **AppID** 和 **Access Token**

### DeepSeek LLM

1. 访问 [DeepSeek 开放平台](https://platform.deepseek.com/)
2. 进入「API Keys」页面
3. 创建新的 API Key

## ✅ 配置检查

运行以下命令检查配置状态：

```bash
cd software
python tools/check_config.py
```

输出示例：

```
==================================================
HomeBot API 密钥配置状态
==================================================

🎤 火山引擎 TTS: ✅ 已配置
   AppID: 870360...
   Access Token: BBsO5****-S-

🤖 DeepSeek LLM: ✅ 已配置
   API Key: sk-523****afef
   Model: deepseek-chat

👁️  图片理解: ✅ 已配置
   Provider: deepseek
   API Key: sk-523****afef

📄 配置文件路径: /path/to/homebot/software/.env.local
==================================================
```

## 🛡️ 安全注意事项

### ⚠️ 永远不要做以下事情

1. **不要将 `.env.local` 提交到 Git**
   - 已添加到 `.gitignore`，但请确认没有强制添加

2. **不要在代码中硬编码密钥**
   ```python
   # ❌ 错误
   API_KEY = "sk-actual-key-here"
   
   # ✅ 正确
   from configs import get_secrets
   api_key = get_secrets().llm.api_key
   ```

3. **不要分享包含密钥的日志**
   - 使用 `check_secrets()` 会自动脱敏显示

### ✅ 安全做法

1. **定期轮换 API Key**
2. **为不同环境使用不同密钥**（开发/测试/生产）
3. **限制 API Key 的权限范围**
4. **监控 API 使用情况**，发现异常及时禁用

## 🔧 开发者说明

### 在代码中使用密钥

```python
from configs import get_config, get_secrets

# 方式一：通过配置对象获取（推荐）
config = get_config()
api_key = config.llm.api_key  # 自动从环境变量加载

# 方式二：直接访问 secrets 模块
secrets = get_secrets()
token = secrets.tts.access_token
```

### 强制检查密钥

在服务启动时强制检查密钥是否配置：

```python
from configs.secrets import require_secrets

def __init__(self):
    require_secrets("tts")  # 如果未配置会打印帮助信息并退出
    # ...
```

### 重新加载配置

```python
from configs import reload_secrets

# 如果用户修改了 .env.local，可以重新加载
reload_secrets()
```

## 📂 文件结构

```
software/
├── .env.example          # 配置模板（可提交到git）
├── .env.local            # 本地配置（gitignored）
├── .env.development      # 开发环境配置（可选）
├── .env.production       # 生产环境配置（可选）
└── src/
    └── configs/
        ├── __init__.py    # 统一导出
        ├── config.py      # 主配置
        └── secrets.py     # 密钥管理
```

## ❓ 常见问题

### Q: 为什么 `.env.local` 不存在程序也能运行？
A: 程序会尝试从环境变量读取密钥。如果都未设置，会在实际需要密钥时才报错。

### Q: 可以在 Docker 中使用吗？
A: 可以，使用 `-e` 参数传递环境变量：
```bash
docker run -e DEEPSEEK_API_KEY=sk-xxx homebot
```

### Q: 如何为不同环境使用不同配置？
A: 创建 `.env.development` 或 `.env.production`，或设置 `ENV` 环境变量切换。

### Q: 图片理解和 LLM 使用相同密钥吗？
A: 默认情况下，如果使用 DeepSeek 作为 Vision Provider，会复用 LLM 的密钥。如需独立配置，设置 `VISION_API_KEY`。

---

如有问题，请参考 [项目 README](../README.md) 或提交 Issue。
