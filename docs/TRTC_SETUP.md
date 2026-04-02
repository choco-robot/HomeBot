# TRTC 音视频配置指南

本文档指导如何配置腾讯云 TRTC（实时音视频）服务，实现远程用户与现场玩家的音视频通话。

## 一、准备工作

### 1. 注册腾讯云账号
1. 访问 [腾讯云官网](https://cloud.tencent.com/)
2. 注册账号并完成实名认证（个人或企业）

### 2. 开通 TRTC 服务
1. 登录腾讯云控制台
2. 搜索并进入「实时音视频 (TRTC)」服务
3. 点击「创建应用」
   - 应用名称：`HomeBot-Mahjong`
   - 应用类型：选择「视频通话」

### 3. 获取关键参数
创建应用后，在应用详情页获取：
- **SDKAppID**（如：1400718166）
- **SDKSecretKey**（如：a1b2c3d4...）

⚠️ **安全提示**：SecretKey 是敏感信息，请勿泄露给他人。

---

## 二、配置方法

### 方法一：使用配置向导（推荐）

运行配置向导脚本：

```powershell
cd software
python tools/setup_trtc.py
```

按提示输入 SDKAppID 和 SecretKey，脚本会自动写入 `.env.local` 文件。

### 方法二：手动配置

1. 打开 `software/.env.local` 文件（如不存在则创建）
2. 添加以下内容：

```ini
# 腾讯云 TRTC 配置
TRTC_SDK_APP_ID=你的_SDKAppID
TRTC_SECRET_KEY=你的_SecretKey
```

3. 保存文件

---

## 三、验证配置

### 1. 验证后端 API
启动麻将 Web 服务：

```powershell
cd software/src
python -m applications.mahjong_bot --host 0.0.0.0 --port 5100
```

在浏览器中访问：
```
http://localhost:5100/api/trtc/usersig?userid=test
```

应返回包含 `userSig` 的 JSON 数据。

### 2. 验证机器人端 TRTC 页面

启动 Chrome Kiosk 模式：

```powershell
start chrome --kiosk --fullscreen --autoplay-policy=no-user-gesture-required "file:///E:/develop/HomeBot/homebot/software/src/applications/mahjong_bot/static/robot_trtc.html"
```

观察状态栏：
- 绿色圆点：连接成功
- 红色圆点：连接失败（检查配置）

### 3. 验证用户端页面

1. 访问 `http://localhost:5100/mahjong`
2. 点击「加入通话」按钮
3. 应能看到机器人端的视频画面

---

## 四、架构说明

### 数据流

```
┌──────────────────┐         ┌──────────────────┐
│   远程用户 Web   │◄───────►│   腾讯云 TRTC    │
│                  │  WebRTC │                  │
└──────────────────┘         └────────┬─────────┘
                                      │
                        ┌─────────────┘
                        │
┌──────────────────┐    │     ┌──────────────────┐
│  robot_trtc.html │◄───┘     │  Mahjong Web     │
│  (Chrome Kiosk)  │◄─────────│  Server (5100)   │
│                  │ 获取配置 │                  │
└──────────────────┘         └──────────────────┘
```

### 角色说明

| 角色 | 说明 | 视频源 |
|------|------|--------|
| 机器人端 | Chrome 全屏运行 `robot_trtc.html` | 前置摄像头 + 顶置摄像头 |
| 远程用户 | 浏览器访问 `mahjong.html` | 用户自己的摄像头 |

---

## 五、常见问题

### Q1: 页面提示 "未配置 TRTC 参数"
- 检查 `.env.local` 文件是否存在
- 检查 `TRTC_SDK_APP_ID` 和 `TRTC_SECRET_KEY` 是否正确填写
- 重启 Mahjong Web 服务

### Q2: 无法获取摄像头
- 确保摄像头已连接并被 Windows 识别
- Chrome 需要授权摄像头权限
- 检查摄像头是否被其他程序占用

### Q3: 音视频卡顿
- 检查网络带宽（建议上行带宽 ≥ 2Mbps）
- 降低视频分辨率（修改 `robot_trtc.html` 中的视频参数）
- 使用有线网络连接

### Q4: 无法听到声音
- 检查音箱/耳机是否连接
- Chrome 是否被静音
- 系统音量设置

---

## 六、安全建议

1. **生产环境必须使用后端生成 UserSig**
   - 前端硬编码 SecretKey 仅用于本地调试
   - 部署时确保 `robot_trtc.html` 能从后端获取配置

2. **定期轮换密钥**
   - 在腾讯云控制台定期更换 SecretKey
   - 更新 `.env.local` 文件

3. **限制 UserSig 有效期**
   - 默认有效期 7 天（604800 秒）
   - 生产环境建议设置为 24 小时
