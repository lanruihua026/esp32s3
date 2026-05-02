# 智能垃圾分类箱 — ESP32-S3 控制板固件

ESP32-S3 作为系统的**控制与网联节点**：接收 ESP32-CAM 的 AI 识别结果，驱动三路舵机完成垃圾分拣，通过三路 HX711 监测仓位重量并在满载时触发声光告警，同时通过 OneNET MQTT 将数据上报至云端供 Web 看板展示。

## 系统架构

```
┌────────────┐  UART(115200)  ┌──────────────────────────────────────┐
│ ESP32-CAM  │ ─────────────> │           ESP32-S3（本板）            │
│  AI 推理   │  DET/NONE/ERR  │                                      │
└────────────┘                │  ┌──────────┐   ┌─────────────────┐  │
                              │  │ 舵机 ×3  │   │   HX711 ×3      │  │
                              │  │ 仓位分拣 │   │   称重告警      │  │
                              │  └──────────┘   └─────────────────┘  │
                              │                                      │
                              │  ┌──────────┐   ┌─────────────────┐  │
                              │  │ OLED 屏  │   │  OneNET MQTT    │  │
                              │  │ 状态显示 │   │  云端上报       │  │
                              │  └──────────┘   └─────────────────┘  │
                              └──────────────────────────────────────┘
                                           │ MQTT (TCP 1883)
                                    ┌──────┴──────┐
                                    │  OneNET 云  │
                                    │  平台       │
                                    └──────┬──────┘
                                           │ HTTP REST
                                    ┌──────┴──────┐
                                    │  Web 看板   │
                                    └─────────────┘
```

## 硬件引脚分配

| 外设 | 信号 | GPIO |
|------|------|------|
| ESP32-CAM 串口 | RX | GPIO 18 |
| ESP32-CAM 串口 | TX | GPIO 17（保留） |
| OLED SH1106 | SDA | GPIO 4 |
| OLED SH1106 | SCL | GPIO 5 |
| 舵机 1（电池仓） | PWM | GPIO 7 |
| 舵机 2（手机仓） | PWM | GPIO 8 |
| 舵机 3（数码配件仓） | PWM | GPIO 16 |
| HX711 — 仓 1 | DT / SCK | GPIO 1 / 2 |
| HX711 — 仓 2 | DT / SCK | GPIO 11 / 12 |
| HX711 — 仓 3 | DT / SCK | GPIO 13 / 14 |
| 按键 1（向左翻页） | — | GPIO 9（内部上拉） |
| 按键 2（向右翻页） | — | GPIO 10（内部上拉） |
| 满载告警灯 | — | GPIO 6 |
| 板载 WS2812 RGB | — | GPIO 38 |

## 快速开始

### 1. 配置凭据

```bash
# 复制模板并填写真实值
cp include/secrets.h.example include/secrets.h
```

编辑 `include/secrets.h`，填入：
- `WIFI_SSID` / `WIFI_PASSWORD` — 本地 WiFi
- `ONENET_PRODUCT_ID` / `ONENET_DEVICE_NAME` / `ONENET_BASE64_KEY` — 在 OneNET 控制台「设备管理」中获取
- `ONENET_TOKEN_EXPIRE_AT` — Token 过期时间戳（Unix 秒），当前值 `1893456000` 对应 2030-01-01

> `secrets.h` 已加入 `.gitignore`，不会提交到版本库。

### 2. 烧录

使用 PlatformIO 编译并烧录：

```bash
pio run --target upload
```

或在 VS Code 中点击底部工具栏「Upload」按钮。

### 3. 串口监控

波特率：**115200**

```bash
pio device monitor --baud 115200
```

## 串口标定命令

开机后通过 USB 串口（115200）发送以下命令，**大小写均可**：

| 命令 | 作用 |
|------|------|
| `TARE1` / `TARE2` / `TARE3` | 对对应仓位执行**去皮**（空仓时发送） |
| `TAREALL` | 三路同时去皮，适合出现空仓固定显示几百克或 1kg 多时快速恢复零点 |
| `CAL1:<重量g>` | 对仓 1 执行**校准**，例如 `CAL1:216`（放 216g 砝码后发送） |
| `CAL2:<重量g>` | 对仓 2 执行校准 |
| `CAL3:<重量g>` | 对仓 3 执行校准 |
| `STATUS` | 打印三路原始 ADC、有效样本数、零点偏移、方向、校准系数和实时重量 |
| `STATUS1` / `STATUS2` / `STATUS3` | 只打印单路调试状态，便于逐个仓位排查 |
| `RAW1` / `RAW2` / `RAW3` | 连续打印单路原始 ADC 采样，检查放重物时 raw 是否明显变化 |
| `RAWALL` | 连续打印三路原始 ADC 采样 |
| `CLEARHX` / `CLEARHX1` / `CLEARHX2` / `CLEARHX3` | 清除三路或单路保存的 HX711 零点、方向和校准系数 |

**标准校准流程（每路独立进行）：**

1. 确认仓位空载稳定 → 发送 `TARE1`
2. 放入已知重量砝码（如 216g）→ 发送 `CAL1:216`
3. 重启设备 → 发送 `STATUS` 确认系数已从 NVS 恢复

> 当前固件启用了上电自动去皮：每次启动时三仓必须保持空载，否则启动时放在仓内的重量会被当作零点。

**硬件响应排查：**

1. 空载发送 `TAREALL`，再发送 `STATUS`，确认三路 `net` 接近 0。
2. 对单路放入同一个已知重量砝码，发送 `RAW1` / `RAW2` / `RAW3`。
3. 正常通道的 raw 应随重量出现数万到十几万级变化；如果只变化几十到几百，优先检查该路传感器、HX711 模块、线序、焊点和机械受力点。

如果调整接线或更换模块后空仓上电仍显示明显重量，先空仓发送 `CLEARHX`，再发送 `TAREALL`，最后用已知砝码重新执行 `CAL1:<重量g>` / `CAL2:<重量g>` / `CAL3:<重量g>`。

## OneNET 物模型属性说明

| 物模型标识符 | 仓位 | 类型 | 读写 | 说明 |
|---|---|---|---|---|
| `phone_weight` | 手机仓（仓 1） | int32 / g | 只读 | 当前重量 |
| `phone_percent` | 手机仓（仓 1） | float / % | 只读 | 满溢百分比 |
| `phone_full` | 手机仓（仓 1） | bool | 只读 | 是否满溢 |
| `mouse_weight` | 数码配件仓（仓 2） | int32 / g | 只读 | 当前重量 |
| `mouse_percent` | 数码配件仓（仓 2） | float / % | 只读 | 满溢百分比 |
| `mouse_full` | 数码配件仓（仓 2） | bool | 只读 | 是否满溢 |
| `battery_weight` | 电池仓（仓 3） | int32 / g | 只读 | 当前重量 |
| `battery_percent` | 电池仓（仓 3） | float / % | 只读 | 满溢百分比 |
| `battery_full` | 电池仓（仓 3） | bool | 只读 | 是否满溢 |
| `overflow_threshold_g` | — | int32 / g | **读写** | 满载阈值（100~5000 g），可远程下发修改 |
| `ai_conf_threshold` | — | float 0~1 | **读写** | AI 置信度阈值，低于此值不触发分拣；0 表示不过滤 |

> 读写属性可通过 OneNET 控制台或 Web 看板下发，设备收到后**立即持久化到 NVS**，重启后自动恢复。

## 垃圾类别与仓位映射

| AI 类别（YOLO 标签） | 仓位 | 舵机 |
|---|---|---|
| `Battery` | 电池仓（仓 3） | GPIO 7（servo1） |
| `MobilePhone` | 手机仓（仓 1） | GPIO 8（servo2） |
| `Charger` / `Earphone` | 数码配件仓（仓 2） | GPIO 16（servo3） |

## OLED 页面说明

按键 1（GPIO9）向左翻页，按键 2（GPIO10）向右翻页，共 3 页循环：

| 页码 | 内容 |
|---|---|
| 1/3 | 系统状态（WiFi / MQTT / HX711 健康状态、AI 置信度阈值） |
| 2/3 | 三仓实时重量与百分比 |
| 3/3 | 最新 AI 识别结果（类别 + 置信度 + 时间戳） |

## 依赖库（PlatformIO 自动安装）

| 库 | 版本 |
|---|---|
| Adafruit SH110X | ^2.1.10 |
| Adafruit GFX Library | ^1.12.4 |
| PubSubClient | ^2.8 |
| Adafruit NeoPixel | ^1.12.3 |
| ArduinoJson | ^7.4.1 |
