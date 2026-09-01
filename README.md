# agilexrobotics

`agilexrobotics` 是面向 AgileX PiPER-X 机械臂的 Python 控制项目，提供 SocketCAN 通信、状态读取、基础运动与夹爪控制，以及供 GELLO 遥操作程序调用的 ZMQ 服务。项目提供两个命令入口：`ag` 用于机械臂调试和控制，`ag-gello-server` 用于启动跟随 GELLO 控制服务。

> **[!WARNING]** 
> 
> - 任何运动命令都可能造成机械臂、夹爪或周围物体损坏。首次使用前请固定机械臂底座、清空工作空间、确保急停按钮触手可及。建议先完成只读状态检查，再进行运动测试。
> 
> - 机械臂失能会导致机械臂从当前状态直接自由落体，可能对机械臂造成不可逆的影响。因此，建议失能前托住机械臂。

## （1）环境要求

- Ubuntu 或其他支持 SocketCAN 的 Linux 系统
- Python 3.11 或更高版本（当前项目使用 `.python-version` 固定为 3.11）
- [Git](https://git-scm.com/) & [uv](https://docs.astral.sh/uv/getting-started/installation/)
- `iproute2` 提供的 `ip` 命令
- PiPER-X 机械臂、兼容的 USB-CAN 适配器和正确的 CAN 总线终端电阻
- LInux 账户能够使用 `sudo` 配置网络接口

## （2）快速开始

### 1. 安装系统工具

在 Ubuntu 上安装 Git、uv 安装脚本所需的 curl，以及配置 SocketCAN 所需的 iproute2：

```bash
sudo apt update
sudo apt install -y git curl iproute2
```

### 2. 获取项目

使用 SSH clone 项目：

```bash
git clone git@github.com:right-or-not/agilexrobotics.git
cd agilexrobotics
```

未配置 GitHub SSH 密钥时可使用 HTTPS：

```bash
git clone https://github.com/right-or-not/agilexrobotics.git
cd agilexrobotics
```

### 3. 安装 uv 和项目依赖

如果系统尚未安装 uv，可使用官方安装脚本：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
```

若安装程序提示了不同的环境加载命令，请以终端提示为准；其他安装方式见 [uv 官方安装说明](https://docs.astral.sh/uv/getting-started/installation/)。安装完成后确认命令可用：

```bash
uv --version
```

安装项目要求的 Python，并根据 `uv.lock` 创建本地 `.venv`、同步依赖：

```bash
uv python install 3.11
uv sync --frozen
```

通常不需要手动激活虚拟环境；后续使用 `uv run ...` 时，uv 会自动使用当前项目的 `.venv`。如需在编辑器或终端中显式激活，可执行：

```bash
source .venv/bin/activate
```

### 4. 连接并配置 CAN

连接 USB-CAN 和 PiPER-X，给机械臂上电并释放急停。先确认系统已经创建 CAN 网络接口：

```bash
ip -brief link show can0
```

将 `can0` 配置为 PiPER-X 使用的 1 Mbit/s，并显示接口统计信息：

```bash
./scripts/config_can.sh
```

脚本会在必要时调用 `sudo`，并可安全地重复执行。默认参数依次为接口名（`interface`）、波特率（`bitrate`）和发送队列长度（`txqueuelen`）；使用其他配置时可显式传入：

```bash
./scripts/config_can.sh can0 1000000 100
```

再次检查接口状态：

```bash
ip -details -statistics link show can0
```

### 5. 进行只读连通性验证

先读取综合状态，该命令不会使能或移动机械臂：

```bash
uv run ag status
```

正常通信时，输出中的 `connected` 和 `communication_ok` 应为 `true`。继续检查关节角和固件信息：

```bash
uv run ag joints
uv run ag firmware
```

如果 `connected` 为 `true` 但 `communication_ok` 为 `false`，通常表示软件已打开 CAN 接口但尚未收到完整反馈；请检查机械臂电源、急停、CAN-H/CAN-L 接线、终端电阻和波特率，并观察 `ip -statistics link show can0` 中的 RX 计数是否增长。

至此，Python 环境、项目依赖和 PiPER-X 只读通信均已完成。运行任何运动命令前，请阅读[开发与调试手册](docs/DEVELOPMENT.md)中的安全说明和命令参数。

## （3）GELLO 服务

完成 CAN 连通性验证后，可启动供 GELLO 客户端连接的本地 ZMQ 服务：

```bash
uv run ag-gello-server --host 127.0.0.1 --port 6001 --hz 50
```

`--hz` 限制 PiPER-X 关节命令的最大发送频率，默认值为 50 Hz。启动服务会进入硬件控制流程；完整的双终端连接步骤、参数含义和退出方式请参阅[开发与调试手册](docs/DEVELOPMENT.md#3-ag-gello-server-与-gello-跟随)。

## （4）开发环境

`uv sync --frozen` 会同步锁文件中声明的开发依赖。常用质量检查命令如下：

```bash
uv run pytest
uv run ruff check .
uv run pyright
```

项目架构、源码职责、CLI 全部参数、硬件调试和 GELLO 联调方法统一记录在[开发与调试手册](docs/DEVELOPMENT.md)中。

## （5）常见问题

### 1. `uv sync` 无法拉取 `pyagxarm`

`pyagxarm` 依赖通过 GitHub Git 仓库安装。请确认当前网络能够访问 GitHub，并且系统已安装 Git，然后重新执行：

```bash
uv sync --frozen
```

### 2. 找不到 `can0`

先确认 USB-CAN 已被系统识别：

```bash
lsusb
ip -brief link show
```

若适配器使用 `gs_usb` 驱动，可尝试加载驱动后重新插拔设备：

```bash
sudo modprobe gs_usb
```

### 3. CAN 接口是 UP，但收不到反馈

接口处于 `UP` 仅表示本机 SocketCAN 已配置完成，不代表机械臂已经发送数据。使用以下命令观察 RX/TX 和 CAN 错误计数，同时检查机械臂供电、急停、接线、终端电阻及 1 Mbit/s 波特率：

```bash
ip -details -statistics link show can0
```

更多诊断方法见[开发与调试手册](docs/DEVELOPMENT.md)。
