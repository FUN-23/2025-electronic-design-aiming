# 2025 电赛自动瞄准装置

## 最终参赛版本

[最终版本代码](final/ypy_key_jiguang_timer_new.py)。针对题目的两秒定时要求，我们采用按键启动后定时 **2 秒强制发射激光**的方案。

`examples/` 中保留团队调试过程的中间版本。

本仓库收录了我们团队在 2025 年电赛项目开发与调试过程中保存的 12 个中间版本脚本，主要实现基于树莓派、Python 和 OpenCV 的矩形靶标识别与双轴电机控制。

这些代码记录了团队在视觉识别、电机控制、按键交互和触发方式等方面的调试与迭代过程，用于展示不同阶段的实现思路，不代表最终参赛版本。源码保留调试时的原始内容，并按功能分类整理，便于回顾与交流。

## 工作原理

摄像头采集图像 → HSV 阈值提取黑色区域 → 轮廓筛选矩形 → 透视变换求中心 → 计算与标定像素坐标的偏差 → 串口控制电机调整方向。

代码采用传统图像处理，不依赖神经网络模型。控制变量采用 PID 命名，但现有配置中积分和微分系数均为零，实际主要使用比例控制及分段限幅。

## 版本导航

| 脚本 | 用途 |
| --- | --- |
| [aaa.py](examples/single_axis/aaa.py) | 基础单轴识别与控制，部分注释存在编码问题 |
| [rectangle_tracker.py](examples/single_axis/rectangle_tracker.py) | 内矩形识别与单轴跟踪 |
| [rectangle_new.py](examples/single_axis/rectangle_new.py) | 矩形搜索与单轴跟踪 |
| [cxd_2.py](examples/dual_axis/cxd_2.py) | 双轴跟踪，目标像素坐标为 (305, 245) |
| [ypy_2.py](examples/dual_axis/ypy_2.py) | 双轴调试版本，目标像素坐标为 (330, 237) |
| [cxd_3_key.py](examples/button_control/cxd_3_key.py) | 双轴跟踪加按键启动 |
| [ypy_2_key.py](examples/button_control/ypy_2_key.py) | 另一套按键启动版本 |
| [ypy_2_key_jiguang.py](examples/aim_trigger/ypy_2_key_jiguang.py) | 双轴误差均小于 5 像素时，触发约 100 ms GPIO 输出 |
| [ypy_key_jiguang_timer.py](examples/timed_trigger/ypy_key_jiguang_timer.py) | 按键后约 4 秒触发输出，包含阻塞等待 |
| [ypy_key_jiguang_timer_new.py](examples/timed_trigger/ypy_key_jiguang_timer_new.py) | 使用独立定时器线程，按键后约 4 秒触发输出 |
| [zise.py](examples/laser_detection/zise.py) | 双轴跟踪并识别、显示紫色激光点 |
| [key_test.py](examples/hardware_tests/key_test.py) | GPIO 17 按键测试 |

建议从 `ypy_key_jiguang_timer_new.py` 阅读整体流程，再与基于误差触发的 `ypy_2_key_jiguang.py` 对照。

## 硬件与依赖

- 支持 `RPi.GPIO` 的树莓派与系统环境。
- OpenCV 可访问的摄像头，脚本通常设置为 640 × 480。
- 双轴版本使用 `/dev/ttyAMA3`（X 轴）和 `/dev/ttyAMA0`（Y 轴），115200 波特率。
- 按键使用 BCM GPIO 17，内部上拉，按下接地。
- 带触发输出的版本使用 BCM GPIO 27；源码标为蜂鸣器，实际连接负载需结合原装置确认。
- 电机驱动器需支持源码中的串口协议；驱动器型号与接线图未包含在原目录中。

在匹配的树莓派环境安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
```

依赖版本未锁定，因为原始代码未附运行环境版本。`RPi.GPIO` 的兼容性取决于树莓派型号和系统。

## 运行

先核对串口、GPIO 接线、电机协议和摄像头，再独立运行所需脚本。程序会直接操作硬件，部分脚本在导入时就初始化串口或 GPIO。

```bash
# 按键测试
python3 examples/hardware_tests/key_test.py

# 按键启动、双轴跟踪、独立定时输出
python3 examples/timed_trigger/ypy_key_jiguang_timer_new.py
```

包含 `cv2.imshow` 的脚本需要图形显示环境。运行前应结合装置重新标定 `TARGET_X`、`TARGET_Y`、颜色阈值、比例系数和电机方向。不同脚本的参数不能直接视为通用值。

## 已知限制

- `timer_new` 的输出由固定 4 秒定时触发，不代表已经瞄准成功。
- `timer_new` 的丢靶重新搜索分支被 `if 0:` 禁用。
- `zise.py` 的紫色光点用于检测和显示，电机控制仍使用矩形中心相对固定目标坐标的误差。
- PID 状态处理尚未按双轴完整分离；如启用积分或微分，需要重新检查实现。
- `aaa.py` 存在编码/注释问题，以实际语法检查结果为准。
- 本次整理未进行摄像头、电机或整机实测，也未验证比赛指标。

源码校验和与静态语法检查结果见 [source-manifest.json](docs/source-manifest.json)。原始 `.idea` 编辑器配置未纳入仓库。题目 PDF 不属于本次代码归档内容。
