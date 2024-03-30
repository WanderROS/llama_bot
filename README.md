<h1 align="center">LlamaBot: Llama ROS Robot</h1>
<div align="center">

> 本项目是一款基于香橙派5（RK3588）的双轮差速 ROS 机器人，使用 RK3588 的 NPU 进行图像识别跟踪，利用 1.28 寸圆形屏幕进行趣味性交互。Llama 大模型在完成基础功能后引入，最终目的打造一款个人家庭小管家（联动 HomeAssistant）。

# OrangePi5 基础环境构建
1. 通过软件烧录香橙派提供的 `Orangepi5_1.1.8_ubuntu_focal_desktop_xfce_linux6.1.43` 固件到 SD 卡中。
2. 修改 orangepi 和 root 账户密码。
3. 删除 xrdp 并安装 NoMachine。