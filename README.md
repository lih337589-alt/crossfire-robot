# Crossfire Robot · 穿越火线智能车

面向 AIC 2026「穿越火线」赛项的自主循迹小车。基于 UVC 摄像头识别红/蓝锥桶，通过有限状态机调度比赛流程，PID 闭环控制舵机与电驱，配合语音播报完成「出发 → 前往据点 → 识别路口 → 穿越雷区 → 返回营地」的全流程。

代码为 C++17 与 C11 混合工程，视觉部分依赖 OpenCV 4，运行平台为 Orange Pi / RK3588 一类 ARM Linux 单板机。

## 硬件平台

| 部件 | 说明 |
| --- | --- |
| 主控 | Orange Pi / RK3588（ARM Linux） |
| 摄像头 | USB UVC，1280×720@30fps |
| 转向/电驱 | PCA9685 I²C PWM 驱动板，舵机 + 电调 |
| 姿态 | 串口 IMU（`/dev/ttyS0`，9600 8N1） |
| 语音 | TTS 模块，串口下发播报文本 |

## 软件架构

```
app/          主程序、任务控制器、硬件自检
config/       配置加载与合法性校验（config.ini）
hal/          硬件抽象层：真实实现与 Fake 实现并存
decision/     有限状态机、事件日志与罚分记录
perception/   视觉管线：颜色分割 → 锥桶检测 → 营地/雷区判定
control/      PID 转向、驱动控制、斜率限幅
drivers/      PCA9685、IMU、PID 的 C 实现
tests/        单元测试、极端场景 Mock、合成视觉场景
docs/         优化改动说明
```

分层的关键在于 `hal/`：`hal_linux.cpp` 面向真机，`hal_fake.cpp` 提供纯软件替身，因此状态机、决策与视觉逻辑可以在没有硬件、甚至没有 OpenCV 的机器上完整测试。

## 比赛流程状态机

```
IDLE → ANNOUNCE → READY → TO_STATION → STATION_STAY
     → ALIGN → CROSSING → TO_CAMP → FINISHED
```

任一安全性故障（丢帧超限、IMU 失联、超时、电压异常）直接转入 `FAILED`；`FINISHED` 与 `FAILED` 均为终态，不可回退。所有状态跃迁写入 CSV 事件日志，附带时间戳与罚分。

## 视觉方案

- HSV 空间双区间红色分割（绕开色相环绕）+ 单区间蓝色分割
- 形态学去噪后按面积、长宽比、填充率、底边宽度做锥桶形状过滤
- 按行聚类求「可通行缺口」，结合红色块位置判断该走路线 A 还是 B
- 所有像素阈值以 640×480 为参考坐标系，加载时按实际分辨率自动缩放
- 颜色卡校准：每场开赛前用场地色卡重新标定阈值

## 控制方案

- 位置式 PID 输出转向修正量，带积分限幅与输出饱和
- 电机与舵机指令均做斜率限幅（`*_SLEW_US_PER_CYCLE`），避免打舵过猛
- 分档速度：巡航 / 接近 / 搜索 / 停止，按视觉状态切换
- 固定时序机动作为无里程计条件下的兜底策略

## 构建

依赖：`g++`、`make`、`pkg-config`、OpenCV 4 开发包。

```bash
sudo apt install -y build-essential pkg-config libopencv-dev v4l-utils i2c-tools
make clean all
```

或用封装脚本（会先检查工具链与 OpenCV，缺失时给出明确报错）：

```bash
./build_prod.sh
```

## 运行

```bash
# 完整比赛流程
./demo_new config/config.ini

# 只做硬件自检：相机、IMU、PWM 通道
./demo_new --self-test config/config.ini
```

## 测试

核心逻辑测试不依赖任何硬件，也不需要 OpenCV（通过 `tests/stub/opencv2` 提供接口替身）：

```bash
./run_tests.sh
```

覆盖七组用例：配置层、状态机、HAL 与控制适配、锥桶纯逻辑、转向控制、极端场景 Mock、IMU 帧解析。若环境装有 OpenCV 4，脚本会额外跑一遍合成视觉场景。

视觉管线单独压测：

```bash
make bench-vision
```

## 配置

所有可调参数集中在 `config/config.ini`，分组为：场地尺寸、相机与 ROI、HSV 阈值、锥桶形状过滤、PWM 脉宽、PID 增益、路线与播报文本、时序机动、营地判定、鲁棒性超时。

加载时会做合法性校验，参数越界直接拒绝启动，避免带病上场。

**上场前必须按实车重新标定**：舵机中位与左右极限、电调怠速、相机曝光、颜色阈值。`config.ini` 中的数值对应某一次具体标定，直接照搬通常不可用。

## 安全提示

- 首次运行请架空车轮再执行 `--self-test`，确认舵机方向与电调中位正确
- 锥桶「命中」判定基于图像坐标近似，不是物理接触传感器，需按实车标定
- 雷区边界的围栏检测同样为视觉代理，不能替代真实防撞

## 说明

仓库中的 `PC_HOST` / `ORANGEPI_IP` 为早期联调遗留的局域网地址，比赛版本不启动 UDP 监听，仅作迁移参考。
