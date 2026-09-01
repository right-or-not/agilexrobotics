# agilexrobotics 开发与调试手册

本文档记录项目架构、命令参数、硬件控制、GELLO 联调和本地开发细节。首次安装和最小连通性验证请先阅读[项目 README](../README.md)。以下命令默认在 `agilexrobotics/` 目录执行。

> 基于官方 `pyAgxArm` SDK，为 PiPER-X 提供 `ag` 命令行控制工具，以及连接 GELLO 主手与 PiPER-X 从臂的 ZMQ 跟随服务。

## （1）项目基本信息

项目使用 Python 3.11 及以上版本，采用 pyenv 安装和选择 Python，采用 uv 创建 `.venv`、管理依赖和运行命令，并使用标准的 `src layout` 组织 Python 包。机械臂通信基于 `pyAgxArm` 和 SocketCAN；GELLO 跟随服务使用 NumPy 处理七维状态，并通过 ZeroMQ 在两个进程之间传输请求和反馈。

### 1. 命令入口

| 命令                       | Python 入口                          | 功能                                        |
| ------------------------ | ---------------------------------- | ----------------------------------------- |
| `uv run ag`              | `agilexrobotics.cli:main`          | 查询状态、配置参数，以及执行 PiPER-X 和 AGX 夹爪控制命令       |
| `uv run ag-gello-server` | `agilexrobotics.gello_server:main` | 将 PiPER-X 封装为 GELLO 可访问的七自由度 ZMQ Robot 服务 |

### 2. 项目架构

项目按“入口与参数解析 → 协议适配 → 安全驱动 → 官方 SDK → 硬件”分层。三条主要调用链如下：

```text
Read command
ag → cli.py → reader.py → pyAgxArm → SocketCAN → PiPER-X / AGX 夹爪

Hardware command
ag → cli.py → driver.py → pyAgxArm → SocketCAN → PiPER-X / AGX 夹爪

GELLO 跟随
GELLO 主手 → piper_x_follow.py → ZMQ → gello_server.py
           → gello_follower.py → driver.py → pyAgxArm → PiPER-X / AGX 夹爪
```

各层职责：

| 层级        | 负责内容                                           |
| --------- | ---------------------------------------------- |
| CLI 层     | 解析参数、区分 Read/Hardware command、校验命令参数并输出 JSON   |
| 只读连接层     | 创建 SDK 对象、管理连接生命周期并整理基础反馈，不使能机械臂               |
| 安全驱动层     | 检查连接、反馈时效、机械臂错误、使能状态、关节范围和单步跨度                 |
| GELLO 适配层 | 在 GELLO 七维状态与 PiPER-X 六轴加 AGX 夹爪之间进行转换         |
| ZMQ 服务层   | 接收 GELLO Robot 协议请求，分派状态读取和七维控制命令              |
| SDK/硬件层   | 由 `pyAgxArm` 通过 `can0` 与 PiPER-X 控制器和 AGX 夹爪通信 |

### 3. 文件结构

```text
agilexrobotics/
├── CLAUDE.md
├── pyproject.toml
├── uv.lock
├── README.md
├── docs/
│   └── DEVELOPMENT.md
├── scripts/
│   └── config_can.sh
├── src/agilexrobotics/
│   ├── __init__.py
│   ├── cli.py
│   ├── driver.py
│   ├── exceptions.py
│   ├── gello_follower.py
│   ├── gello_server.py
│   └── reader.py
└── tests/
    ├── test_cli.py
    ├── test_driver.py
    ├── test_gello_follower.py
    └── test_reader.py
```

### 4. 文件功能

| 文件                                   | 实现的功能                                                    |
| ------------------------------------ | -------------------------------------------------------- |
| `CLAUDE.md`                          | 保存本地开发协作说明，不参与安装、运行或硬件控制                                 |
| `pyproject.toml`                     | 定义项目元数据、Python 版本、依赖、开发工具，以及 `ag`、`ag-gello-server` 命令入口 |
| `uv.lock`                            | 锁定直接和间接依赖版本，供 `uv sync` 创建可复现环境                          |
| `README.md`                          | 项目简介，以及从克隆仓库到首次连通性验证的快速开始                                |
| `docs/DEVELOPMENT.md`                | 项目架构、完整命令参数、硬件控制和 GELLO 联调说明                             |
| `scripts/config_can.sh`              | 检查接口与权限，并完成 SocketCAN 接口关闭、波特率设置、启动和状态输出                 |
| `src/agilexrobotics/__init__.py`     | 标记 `agilexrobotics` Python 包；当前不额外导出公共对象                 |
| `src/agilexrobotics/cli.py`          | 定义全部 `ag` 参数和命令；按常用/不常用、Read/Hardware 分类分派并输出结果          |
| `src/agilexrobotics/reader.py`       | 提供轻量只读连接；读取固件、关节、末端位姿和综合快照                               |
| `src/agilexrobotics/driver.py`       | 提供带安全校验的状态、使能、运动、配置、错误恢复和 AGX 夹爪控制                       |
| `src/agilexrobotics/exceptions.py`   | 定义通信、反馈、机械臂状态、未使能和关节命令等项目异常                              |
| `src/agilexrobotics/gello_follower.py` | 实现 GELLO Robot 适配器；组合六轴与夹爪状态，并正向映射夹爪开口                 |
| `src/agilexrobotics/gello_server.py` | 建立 ZMQ REP 服务，分派 GELLO 请求，并负责启动和关闭 PiPER-X 适配器           |
| `tests/test_cli.py`                  | 验证参数、别名、命令分派、返回结果和运动控制路径                                 |
| `tests/test_driver.py`               | 验证连接、反馈、安全限制、使能、运动、配置和夹爪对象复用                             |
| `tests/test_gello_follower.py`       | 验证七维 GELLO 协议、启停、夹爪正向映射和反馈缺失降级                           |
| `tests/test_reader.py`               | 验证只读连接生命周期、反馈转换和 SDK 对象创建                                |

## （2）安装与 CAN 配置

首次安装步骤见[项目 README](../README.md)。开发环境中读取 `.python-version`，由 pyenv 准备解释器，再将其显式交给 uv：

```bash
cd agilexrobotics
requested_version="$(<.python-version)"
resolved_version="$(pyenv latest -k "$requested_version")"
pyenv install -s "$resolved_version"
interpreter="$(PYENV_VERSION="$resolved_version" pyenv which python)"
UV_NO_MANAGED_PYTHON=1 uv sync --frozen --python "$interpreter"
```

配置默认 CAN 接口 `can0`，波特率为 `1,000,000 bit/s`：

```bash
./scripts/config_can.sh
```

脚本会依次关闭接口、设置 CAN 波特率、重新启用接口并输出接口状态。普通用户执行时会自动使用 `sudo`。脚本没有设置 `restart-ms`，因此也兼容不支持 Bus-Off 自动重启的 CAN 设备。

需要使用其他接口、波特率或发送队列长度时，可传入三个位置参数：

```bash
./scripts/config_can.sh can1 500000 100
```

| 位置参数         | 含义         | 默认值             | 取值要求     |
| ------------ | ---------- | --------------- | -------- |
| `INTERFACE`  | CAN 网络接口名  | `can0`          | 接口必须真实存在 |
| `BITRATE`    | CAN 波特率    | `1000000 bit/s` | 正整数      |
| `TXQUEUELEN` | CAN 发送队列长度 | `100`           | 正整数      |

配置完成后可用以下命令验证机械臂反馈：

```bash
uv run ag status
```

## （3）`ag` 命令详解

基本格式：

```bash
uv run ag COMMAND [OPTIONS]
```

所有命令都会以 JSON 输出结果。成功返回退出码 `0`；连接、反馈、参数或硬件操作失败返回 `1`；只读接口在等待时间内没有收到反馈时返回 `2`。

### 1. 参数解析

| 参数                        | 含义                        | 默认值          | 单位/范围                          |
| ------------------------- | ------------------------- | ------------ | ------------------------------ |
| `command`                 | 要执行的命令                    | 无，必填         | 见本章第 2、3 节                  |
| `--channel`               | SocketCAN 通道              | `can0`       | 有效 CAN 接口名                     |
| `--interface`             | python-can 后端             | `socketcan`  | 后端名称                           |
| `--firmware`              | 主控制器固件协议族                 | `default`    | `default/v183/v188/v189`       |
| `--wait`                  | 连接后等待首批周期反馈的时间            | `0.2`        | 秒，≥ 0                          |
| `--timeout`               | 查询或标定反馈的最长等待时间            | `1.0`        | 秒，≥ 0                          |
| `--speed-percent`         | 规划运动速度百分比                 | `20`         | `%`，整数 1～100                   |
| `--joint`                 | 指定单个关节                    | 全部或未指定       | J1～J6，对应整数 1～6                 |
| `--delta-deg`             | 单关节相对转动量                  | 无，使用时必填      | 度，非 0，范围 −90～90                |
| `--joints`                | J1～J6 的绝对目标角              | 无，使用时必填      | 6 个有限数，单位 rad                  |
| `--pose`                  | `X Y Z Roll Pitch Yaw` 位姿 | 无，使用时必填      | XYZ：m；RPY：rad                  |
| `--mid`                   | 圆弧途经点位姿                   | 无，使用时必填      | 6 个数，单位同 `--pose`              |
| `--end`                   | 圆弧终点位姿                    | 无，使用时必填      | 6 个数，单位同 `--pose`              |
| `--rating`                | 碰撞保护或助力等级                 | 省略时只查询       | `protect`：0～8<br>`assist`：0～10 |
| `--width`                 | AGX 夹爪目标开口宽度              | 无，使用时必填      | m，非负有限数                        |
| `--force`                 | AGX 夹爪目标夹持力               | `1.0`        | N，非负有限数                        |
| `--payload`               | 末端负载档位                    | `empty`      | `empty/half/full`              |
| `--position`              | 底座安装方向                    | `horizontal` | `horizontal/left/right`        |
| `--confirm-hardware-test` | 旧版脚本兼容开关                  | `False`      | 无实际控制作用                        |

参数可以写在命令前后，但推荐统一写在命令后。以下“完整示例”会显式写出该命令真正使用的全部参数；没有可调参数的命令只保留一个推荐示例。

### 2. Read command

Read command 只读取反馈或查询配置，不使能机械臂、不修改控制器配置，也不发送运动命令。其中 `status`、`joints` 和 `firmware` 使用轻量只读连接，其余命令通过 `PiperXDriver` 完成更完整的反馈校验。

#### 常用指令

| 命令       | 功能                             | 最简示例               | 完整示例                                                                                                |
| -------- | ------------------------------ | ------------------ | --------------------------------------------------------------------------------------------------- |
| `status` | 汇总连接、通信、FPS、固件、六轴角度、末端位姿和控制器状态 | `uv run ag status` | `uv run ag status --channel can0 --interface socketcan --firmware default --wait 0.2 --timeout 1.0` |
| `joints` | 读取 J1～J6 当前反馈角度，单位 rad         | `uv run ag joints` | `uv run ag joints --channel can0 --interface socketcan --firmware default --wait 0.2`               |
| `fps`    | 查看 SDK 接收 CAN 周期反馈的频率，单位 Hz    | `uv run ag fps`    | `uv run ag fps --channel can0 --interface socketcan --firmware default --wait 0.2`                  |

#### 不常用指令

| 命令                | 功能                            | 最简示例                      | 完整示例                                                                                                  |
| ----------------- | ----------------------------- | ------------------------- | ----------------------------------------------------------------------------------------------------- |
| `firmware` / `fw` | 查询主控制器固件信息                    | `uv run ag firmware`      | `uv run ag firmware --channel can0 --interface socketcan --firmware default --wait 0.2 --timeout 1.0` |
| `driver-status`   | 读取经过通信、格式、时效和错误检查的完整状态        | `uv run ag driver-status` | `uv run ag driver-status --channel can0 --interface socketcan --firmware default --wait 0.2`          |
| `motor-status`    | 读取六轴整体状态，以及逐电机的位置、电流、温度、电压和故障 | `uv run ag motor-status`  | `uv run ag motor-status --channel can0 --interface socketcan --firmware default --wait 0.2`           |
| `grip-status`     | 读取 AGX 夹爪开口、力、模式、使能、回零和错误状态   | `uv run ag grip-status`   | `uv run ag grip-status --channel can0 --interface socketcan --firmware default --wait 0.2`            |
| `pose`            | 读取法兰末端六维位姿和六轴角度               | `uv run ag pose`          | `uv run ag pose --channel can0 --interface socketcan --firmware default --wait 0.2`                   |
| `limits`          | 查询每个关节及末端当前运动限制               | `uv run ag limits`        | `uv run ag limits --channel can0 --interface socketcan --firmware default --wait 0.2 --timeout 1.0`   |
| `ratings`         | 查询各关节的碰撞保护和助力等级               | `uv run ag ratings`       | `uv run ag ratings --channel can0 --interface socketcan --firmware default --wait 0.2 --timeout 1.0`  |

### 3. Hardware command

Hardware command 使用带反馈校验和安全限制的 `PiperXDriver`。部分命令会使能机械臂、修改固件配置或产生真实运动，执行前必须确认机械臂周围没有人员和障碍物。

表格中的“SDK”表示该 `ag` 命令最终调用的底层 pyAgxArm 运动接口；用户执行时必须使用第一行显示的 `ag` 命令名。

#### 常用指令

| `ag` 命令 / SDK 方法  | 功能                                 | 最简示例                                                                                                   | 完整示例                                                                                                                                                                                         |
| ----------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `enable` / `on`   | 使能 J1～J6，使机械臂可以接收运动命令；不会额外发送位置目标   | `uv run ag enable`                                                                                     | `uv run ag enable --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                                                     |
| `disable` / `off` | 失能 J1～J6；机械臂可能因重力下坠                | `uv run ag disable`                                                                                    | `uv run ag disable --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                       |
| `zero`            | 将 J1～J6 移动到当前坐标系的 `0 rad`；不会重新标定零点 | `uv run ag zero`                                                                                       | `uv run ag zero --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                                                       |
| `stop`            | 发送带阻尼电子急停并撤销驱动的已使能记录；不是物理断电        | `uv run ag stop`                                                                                       | `uv run ag stop --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                          |
| `reset`           | 恢复控制器运动状态；不是断电重启、恢复出厂或机械臂回零        | `uv run ag reset`                                                                                      | `uv run ag reset --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                         |
| `grip`            | 将 AGX 夹爪移动到指定开口宽度并设置夹持力            | `uv run ag grip --width 0.035`                                                                         | `uv run ag grip --width 0.035 --force 1.0 --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                |
| `grip-zero`       | 把 AGX 夹爪当前位置标定为零点，并等待夹爪确认          | `uv run ag grip-zero`                                                                                  | `uv run ag grip-zero --timeout 1.0 --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                       |
| `grip-reset`      | 复位 AGX 夹爪控制器；不会复位六轴或标定夹爪零点         | `uv run ag grip-reset`                                                                                 | `uv run ag grip-reset --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                    |
| `movej`           | 以普通关节模式移动到六轴绝对目标，并逐段等待到位           | `uv run ag movej --joints 0 0 0 0 0 0`                                                                 | `uv run ag movej --joints 0 0 0 0 0 0 --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                                 |
| `movejs`          | 以高响应、无平滑规划模式持续刷新六轴绝对目标直到到位，可能产生冲击  | `uv run ag movejs --joints 0 0 0 0 0 0`                                                                | `uv run ag movejs --joints 0 0 0 0 0 0 --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                                |
| `movep`           | 使用 P 模式移动到指定末端位姿                   | `uv run ag movep --pose 0.20 0 0.30 0 1.57 0`                                                          | `uv run ag movep --pose 0.20 0 0.30 0 1.57 0 --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                          |
| `movel`           | 使用直线 L 模式移动到指定末端位姿                 | `uv run ag movel --pose 0.20 0 0.30 0 1.57 0`                                                          | `uv run ag movel --pose 0.20 0 0.30 0 1.57 0 --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                          |
| `movec`           | 按起点、途经点和终点执行末端圆弧运动                 | `uv run ag movec --pose 0.20 0 0.30 0 1.57 0 --mid 0.20 0.05 0.35 0 1.57 0 --end 0.20 0 0.40 0 1.57 0` | `uv run ag movec --pose 0.20 0 0.30 0 1.57 0 --mid 0.20 0.05 0.35 0 1.57 0 --end 0.20 0 0.40 0 1.57 0 --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20` |

> **当前未实现：** pyAgxArm 的单关节 MIT 控制方法 `move_mit` 尚未封装到
> `PiperXDriver` 和 `ag` CLI，因此目前不能执行 `uv run ag move_mit`。如需将它
> 作为常用指令，必须先补充驱动方法、CLI 参数与安全校验。

#### 不常用指令

| 命令           | 功能                                              | 最简示例                                           | 完整示例                                                                                                                                                                                                                   |
| ------------ | ----------------------------------------------- | ---------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `hold`       | 使能后把当前实测六轴位置设为目标，保持当前位置                         | `uv run ag hold`                               | `uv run ag hold --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                                                                                 |
| `move-joint` | 让指定关节相对当前位置转动一次，其余关节保持当前目标                      | `uv run ag move-joint --joint 1 --delta-deg 5` | `uv run ag move-joint --joint 1 --delta-deg 5 --channel can0 --interface socketcan --firmware default --wait 0.2 --speed-percent 20`                                                                                   |
| `clear`      | 清除指定关节错误；省略 `--joint` 时清除全部关节错误                 | `uv run ag clear`                              | `uv run ag clear --joint 1 --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                                         |
| `max-limits` | 将 J1～J6 限值写为 `3.0 rad/s` 和 `5.0 rad/s²`，并要求固件确认 | `uv run ag max-limits`                         | `uv run ag max-limits --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                                              |
| `defaults`   | 只恢复关节角度、速度和加速度默认限制，不恢复全部出厂参数                    | `uv run ag defaults`                           | `uv run ag defaults --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                                                |
| `speed`      | 单独写入后续规划运动使用的速度百分比                              | `uv run ag speed`                              | `uv run ag speed --speed-percent 20 --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                                |
| `payload`    | 设置控制器使用的末端负载档位                                  | `uv run ag payload`                            | `uv run ag payload --payload full --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                                  |
| `install`    | 设置机械臂底座的实际安装方向                                  | `uv run ag install`                            | `uv run ag install --position right --channel can0 --interface socketcan --firmware default --wait 0.2`                                                                                                                |
| `protect`    | 不带 `--rating` 时查询等级；带参数时设置单轴或全部关节碰撞保护等级         | `uv run ag protect`                            | 查询：`uv run ag protect --channel can0 --interface socketcan --firmware default --wait 0.2 --timeout 1.0` 设置：`uv run ag protect --joint 1 --rating 8 --channel can0 --interface socketcan --firmware default --wait 0.2` |
| `assist`     | 不带 `--rating` 时查询等级；带参数时设置单轴或全部关节助力等级           | `uv run ag assist`                             | 查询：`uv run ag assist --channel can0 --interface socketcan --firmware default --wait 0.2 --timeout 1.0` 设置：`uv run ag assist --joint 1 --rating 10 --channel can0 --interface socketcan --firmware default --wait 0.2`  |

`zero`、`movej` 和 `movejs` 会按最大关节角变化自动拆分路径点，每个路径点最多等待 10 秒，并要求目标误差不超过 `0.1°`。命令结束后只断开主机连接，不会自动失能机械臂。

## （4）`ag-gello-server` 与 GELLO 跟随

`ag-gello-server` 将 PiPER-X 暴露为 GELLO 的七自由度 ZMQ Robot：前六维是 J1～J6，第七维是归一化夹爪位置。夹爪约定为 `0=全闭`、`1=全开`，默认采用正向映射：

```text
AGX width_m = GELLO gripper × 0.1
```

夹爪反馈暂时缺失时，服务会沿用最近一次成功发送的夹爪命令，不会因此中止六轴跟随。客户端完成启动对齐时，会把 GELLO 当前第七维位置立即发送给服务端，因此连接成功后机械臂夹爪会先同步到 GELLO 夹爪的当前开合位置。持续跟随时，客户端只对 J1～J6 应用最高 `1.0 rad` 的单步保护，夹爪第七维直接透传，不参与六轴缩放；服务端使用 `move_js`，并保留 `1.0 rad` 的最终单步安全检查。

### 1. 服务端参数

```bash
uv run ag-gello-server [OPTIONS]
```

| 参数                                                           | 含义                   | 默认值         | 单位/范围                                   |
| ------------------------------------------------------------ | -------------------- | ----------- | --------------------------------------- |
| `--channel`                                                  | PiPER-X CAN 通道       | `can0`      | 有效 CAN 接口名                              |
| `--interface`                                                | python-can 后端        | `socketcan` | 后端名称                                    |
| `--firmware`                                                 | 主控制器固件协议族            | `default`   | 固件名称                                    |
| `--host`                                                     | ZMQ 监听地址             | `127.0.0.1` | IP 或主机名                                 |
| `--port`                                                     | ZMQ 监听端口             | `6001`      | 整数端口                                    |
| `--hz`                                                       | PiPER-X JS 命令的最大发送频率 | `50`        | Hz，正有限数                                 |
| `--gripper-max-width-m`                                      | AGX 夹爪全开宽度           | `0.1`       | m，正有限数                                  |
| `--gripper-force-n`                                          | 跟随时使用的夹持力            | `1.0`       | N，非负有限数                                 |
| `--configure-motion-limits` / `--no-configure-motion-limits` | 是否在启动时写入最大速度和加速度限制   | 开启          | 默认写入 `3.0 rad/s`、`5.0 rad/s²`；固件不支持时可关闭 |
| `--confirm-hardware-control`                                 | 旧版兼容参数               | 关闭          | 无实际控制作用                                 |

服务端启动时不会立即进入 JS 模式；收到客户端第一帧跟随目标时，才建立 `move_js` 流式会话并设置一次运动模式，后续直接高频刷新目标。验证结果表明默认六轴符号为 `1 1 -1 -1 1 1`，其中 J3 和 J4 使用反向映射。服务端不提供也不写入 `--speed-percent`；JS 跟随速度由 GELLO 指令频率、目标变化量以及机械臂固件的最大速度和加速度决定。启动阶段也不发送普通 `move_j` 保持命令，以免重启服务时把仍处于 JS 模式的控制器切换到普通 J 模式。

推荐使用默认配置：

```bash
uv run ag-gello-server
```

需要显式配置全部有效参数时：

```bash
uv run ag-gello-server \
  --channel can0 \
  --interface socketcan \
  --firmware default \
  --host 127.0.0.1 \
  --port 6001 \
  --hz 50 \
  --gripper-max-width-m 0.1 \
  --gripper-force-n 1.0 \
  --configure-motion-limits
```

### 2. 主要函数

| 函数/类                                     | 功能                                                          |
| ---------------------------------------- | ----------------------------------------------------------- |
| `gello_server._parser()`                 | 解析本章第 1 节中的服务端参数                                          |
| `gello_server._dispatch()`               | 分派 GELLO 的自由度、关节状态、关节命令和观测请求                                |
| `gello_server.main()`                    | 创建 PiPER-X 适配器和 ZMQ 服务，并处理 `SIGINT/SIGTERM` 退出              |
| `GelloPiperXRobot.start()`               | 连接、配置限位、使能并读取七维状态；不发送速度或运动模式命令                              |
| `GelloPiperXRobot.get_observations()`    | 返回七维关节状态/速度、末端位姿和夹爪位置                                       |
| `GelloPiperXRobot.command_joint_state()` | 前六维按 `--hz` 限频后使用 `move_js` 流式发送给 PiPER-X；第七维独立正向映射到 AGX 夹爪 |
| `GelloPiperXRobot.close()`               | 断开 CAN；默认不会主动失能机械臂                                          |

ZMQ 协议使用 Python `pickle`，只能绑定并开放给可信客户端。

### 3. 两个终端的职责

| 终端   | 所在项目             | 负责的设备和功能                                                  |
| ---- | ---------------- | --------------------------------------------------------- |
| 终端 1 | `agilexrobotics` | 独占 PiPER-X 的 CAN 连接；使能从臂并提供 `tcp://127.0.0.1:6001` ZMQ 服务 |
| 终端 2 | `gello_software` | 独占 GELLO 的 FTDI/Dynamixel 串口；读取主手并把七维目标发送给终端 1            |

同一时间不要启动第二个占用 `can0` 的机械臂程序，也不要让多个进程占用 GELLO 串口。

### 4. 使用流程

1. 固定 PiPER-X，确认 CAN 已配置，GELLO 串口设备已连接。

2. 在终端 1 启动 PiPER-X 服务端：
   
   ```bash
   cd /path/to/agilexrobotics
   uv run ag-gello-server
   ```
   
   出现以下信息后再启动终端 2：
   
   ```text
   PiPER-X GELLO server listening on tcp://127.0.0.1:6001
   ```

3. 在终端 2 启动 GELLO 主手客户端：
   
   ```bash
   cd ~/projects/gello_software
   .venv/bin/python experiments/piper_x_follow.py \
     --gello-port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBM4Z46-if00-port0 \
     --start-joints 0 0 0 0 0 0
   ```
   
   `--start-joints` 提供六个用于 Dynamixel 多圈角度对齐的参考值；如果当前机械臂不在零位，可先执行 `uv run ag joints`，把返回的 J1～J6 弧度值填入该参数。客户端会使用启动时的相对姿态对齐，保留 GELLO 第七维夹爪命令，并限制单次目标变化。

4. 看到以下输出后才表示跟随循环已经启动：
   
   ```text
   GELLO and PiPER-X aligned; teleoperation started (Ctrl-C to stop)
   ```

5. 停止时先在终端 2 按 `Ctrl-C` 停止主手命令，再在终端 1 按 `Ctrl-C` 关闭 ZMQ/CAN 服务。服务默认只断开连接而不失能机械臂；如需失能，请支撑机械臂后执行 `uv run ag disable`。

若 GELLO 偶尔打印 `warning, comm failed: -3001`，表示 Dynamixel 状态包接收超时。偶发一次会跳过当前帧并继续运行；持续出现时应检查 FTDI/串联线、供电、USB Hub 和串口是否被其他进程占用。
