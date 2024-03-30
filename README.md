<h1 align="center">LlamaBot: Llama ROS Robot</h1>
<div align="center">

> 本项目是一款基于香橙派5（RK3588）的双轮差速 ROS 机器人，使用 RK3588 的 NPU 进行图像识别跟踪，利用 1.28 寸圆形屏幕进行趣味性交互。Llama 大模型在完成基础功能后引入，最终目的打造一款个人家庭小管家（联动 HomeAssistant）。

# OrangePi5 基础环境
1. 通过软件烧录香橙派提供的 `Orangepi5_1.1.8_ubuntu_focal_desktop_xfce_linux6.1.43` 固件到 SD 卡中。

2. 修改 orangepi 和 root 账户密码（安全）。

3. 远程登录 Linux 系统桌面：

   - 配置 OrangePi 5 的 xrdp 后，远程桌面登录后没有任何内容展示

   - 参考用户手册使用 NoMachine 远程登录，下载 NoMachine 的 [ARMv8 安装包](https://downloads.nomachine.com/download/?id=115&distro=ARM)，上传到 OrangePi 5 中

     ```shell
     sudo tar zvxf nomachine_8.11.3_3_aarch64.tar.gz -C /opt/ 
     ```

   - 进入 `/opt/NX` 目录，执行如下命令安装 NoMachine ：

     ```shell
     sudo ./nxserver --install
     ```

   - 安装完成后，下载 [Mac](https://downloads.nomachine.com/download/?id=7) NoMachine 软件，通过 Mac 远程访问 OrangePi 5 桌面

   - 删除默认安装的 xrdp：

     ```shell
     sudo apt remove xrdp
     ```

4. 修改 `/boot/orangepiEnv.txt` ，添加如下内容：

   ```shell
   overlays=spi4-m0-cs1-spidev uart0-m2 uart1-m1 uart4-m0
   ```

   - 开启 SPI4、UART0、UART1、UART4
   - SPI4 用于外接 1.28 存圆形彩屏
   - UART0、UART1、UART4 用于外接底盘主控、激光雷达等串口设备

5. OrangePi 5 上安装了 adb 服务进程，在 Mac 上也可以通过 adb 来连接 OrangePi 5：

   1. 在 Mac 上安装 adb 工具包：

      ```shell
      brew install android-platform-tools --cask
      ```

   2. adb 网络连接 OrangePi 5：

      ```shell
      adb connect orange.local:5555
      ```

   3. 远程 shell OrangePi 5：

      ```shell
      adb shell
      ```

6. 安装 ROS1：

   ```shell
   install_ros.sh ros1
   ```

7. 安装 RKNN：
   1. 更新以及安装软件：

      ```shell
      sudo apt-get update
      sudo apt-get install git
      ```

   2. 下载官方提供的 rknpu2代码并拷贝相应的文件到 OrangePi 5 系统目录下：

      ```shell
      git clone https://github.com/rockchip-linux/rknpu2.git
      cd rknpu2
      sudo cp ./runtime/RK3588/Linux/librknn_api/aarch64/* /usr/lib/
      sudo cp ./runtime/RK3588/Linux/librknn_api/include/* /usr/include/
      sudo cp ./runtime/RK3588/Linux/rknn_server/aarch64/usr/bin/* /usr/bin/
      
      ```

   3. 安装 Python 依赖包：

      ```shell
      sudo apt-get install -y python3-dev
      sudo apt-get install -y python3-pip
      sudo apt-get install -y python3-numpy
      sudo apt-get install -y python3-opencv
      ```

   4. 下载 RKNN-toolkit 代码以及安装 RKNN-toolkit：

      ```shell
      git clone https://github.com/rockchip-linux/rknn-toolkit2.git
      cd rknn-toolkit2
      cd rknn_toolkit_lite2/packages/
      # 确认 Python 版本 python3 --version => Python 3.8.10
      pip3 install ./rknn_toolkit_lite2-1.6.0-cp38-cp38-linux_aarch64.whl -i https://pypi.tuna.tsinghua.edu.cn/simple/
      ```

   5. 验证 RKNN 的安装：

      ```shell
      cd rknn_toolkit_lite2/examples/dynamic_shape
      python3 test.py
      # 输出
      --> Load RKNN model
      done
      --> Init runtime environment
      I RKNN: [20:31:28.584] RKNN Runtime Information: librknnrt version: 1.5.2 (c6b7b351a@2023-08-23T15:28:22)
      I RKNN: [20:31:28.584] RKNN Driver Information: version: 0.9.3
      I RKNN: [20:31:28.584] RKNN Model Information: version: 6, toolkit version: 1.6.0+81f21f4d(compiler version: 1.6.0 (585b3edcf@2023-12-11T07:42:56)), target: RKNPU v2, target platform: rk3588, framework name: Caffe, framework layout: NCHW, model inference type: dynamic_shape
      W RKNN: [20:31:28.584] RKNN Model version: 1.6.0 not match with rknn runtime version: 1.5.2
      done
      --> Running model
      model: mobilenet_v2
      
      input shape: 1,3,224,224
      W The input[0] need NHWC data format, but NCHW set, the data format and data buffer will be changed to NHWC.
      W RKNN: [20:31:28.602] Output(prob): size_with_stride larger than model origin size, if need run OutputOperator in NPU, please call rknn_create_memory using size_with_stride.
      -----TOP 5-----
      [155] score:0.936035 class:"Shih-Tzu"
      [204] score:0.002516 class:"Lhasa, Lhasa apso"
      [154] score:0.002172 class:"Pekinese, Pekingese, Peke"
      [283] score:0.001601 class:"Persian cat"
      [284] score:0.000286 class:"Siamese cat, Siamese"
      
      input shape: 1,3,160,160
      W The input[0] need NHWC data format, but NCHW set, the data format and data buffer will be changed to NHWC.
      W RKNN: [20:31:28.606] Output(prob): size_with_stride larger than model origin size, if need run OutputOperator in NPU, please call rknn_create_memory using size_with_stride.
      -----TOP 5-----
      [155] score:0.606934 class:"Shih-Tzu"
      [154] score:0.329834 class:"Pekinese, Pekingese, Peke"
      [204] score:0.025085 class:"Lhasa, Lhasa apso"
      [194] score:0.001038 class:"Dandie Dinmont, Dandie Dinmont terrier"
      [219] score:0.000241 class:"cocker spaniel, English cocker spaniel, cocker"
      
      input shape: 1,3,256,256
      W The input[0] need NHWC data format, but NCHW set, the data format and data buffer will be changed to NHWC.
      W RKNN: [20:31:28.614] Output(prob): size_with_stride larger than model origin size, if need run OutputOperator in NPU, please call rknn_create_memory using size_with_stride.
      -----TOP 5-----
      [155] score:0.927246 class:"Shih-Tzu"
      [154] score:0.007225 class:"Pekinese, Pekingese, Peke"
      [204] score:0.004616 class:"Lhasa, Lhasa apso"
      [193] score:0.000878 class:"Australian terrier"
      [283] score:0.000482 class:"Persian cat"
      
      done
      ```

8. YOLOv5 在 RK3588 上运行需要安装 mpp 和 rga 这两个三方依赖。进入 `rknpu2/examples/3rdparty` 目录下可以看到 mpp 和 rga 文件夹。

9. 配置 RK3588 上的 mpp 环境：
   1. 进入 `rknpu2/examples/3rdparty/mpp` 目录：

       ```shell
       cd rknpu2/examples/3rdparty/mpp
       ```

   2. 拷贝 `include` 目录下的头文件到 `/usr/include/` 目录下：

       ```shell
       sudo cp -r include/* /usr/include/
       ```

   3. 拷贝动态库文件到 `/usr/lib/` 目录下：

       ```shell
       sudo cp Linux/aarch64/* /usr/lib/
       ```

10. 配置 RK3588 上的 rga 环境：

    1. 进入 `rknpu2/examples/3rdparty/rga/RK3588` 目录：

       ```shell
       cd rknpu2/examples/3rdparty/rga/RK3588
       ```

    2. 拷贝 `include` 目录下的头文件到 `/usr/include/rga` 目录下：

       ```shell
       sudo cp include/* /usr/include/rga/
       ```

    3. 拷贝动态库文件到 `/usr/lib/` 目录下：

       ```shell
       sudo cp lib/Linux/aarch64/* /usr/lib/
       ```