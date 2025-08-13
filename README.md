<div align="center">

# 倒立摆控制库 (Inverted Pendulum Control Library)

**一个高效、易用的倒立摆控制与研究平台**

</div>

<div align="center">

[![Python Version](https://img.shields.io/badge/Python-3.7+-blue.svg?style=for-the-badge&logo=python)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg?style=for-the-badge&logo=githubactions)](https://github.com/)
[![Issues](https://img.shields.io/github/issues/your-username/your-repo?style=for-the-badge&logo=github)](https://github.com/your-username/your-repo/issues)

</div>

> **✨ 联合开发** <br>
> 本项目由 **卓益得 (droid)** 与 **华为昇腾 (Huawei Ascend)** 团队联合开发，致力于封装与倒立摆相关的通信协议，并提供稳定可靠的PID控制算法。

---

### 🚀 项目展示 (Showcase)
<table>
  <tr>
    <td align="center">
      <img src="resources/outlook.jpg" alt="项目硬件外观" width="320">
      <br>
      <sub><b>硬件整体外观</b></sub>
    </td>
    <td align="center">
      <img src="resources/waic_1.jpg" alt="2025世界人工智能大会展示" width="320">
      <br>
      <sub><b>2025世界人工智能大会</b></sub>
    </td>
    <td align="center">
      <img src="resources/inner.jpg" alt="项目硬件内饰" width="320">
      <br>
      <sub><b>高集成度内部结构</b></sub>
    </td>

  </tr>
</table>

---

### 📚 **目录 (Table of Contents)**
* [核心特性](#-核心特性-features)
* [技术栈](#-技术栈-tech-stack)
* [安装指南](#-安装指南-installation)
* [快速开始](#-快速开始-quick-start)
* [MuJoCo 仿真](#-mujoco-仿真-simulation)
* [视频演示](#-视频演示-demo)
* [参与贡献](#-参与贡献-contributing)


---

### 🌟 核心特性 (Features)

| 特性图标 | 名称 | 详细描述 |
| :---: | :--- | :--- |
| **✨** | **硬件抽象层** | 解耦上层应用与底层硬件。用户无需关心串口通信协议，只需调用几个函数，即可搭建倒立摆 `env` (环境)并与之交互。 |
| **🎮** | **多模态控制** | 支持直立环和位置环双闭环控制，以获得更好的控制效果，确保系统在各种扰动下的稳定与精准。 |
| **⚖️** | **智能状态切换** | 内置“平衡”(Balance)、“摇摆”(Swing up)、“过渡”(Transition)和“响应”(PID)多种模式的自动切换，以适应不同工况。 |
| **📂** | **便捷数据记录** | 自动将控制器在运行过程中的数据保存至CSV文件，便于后续使用 Python 等工具进行离线分析和算法优化。 |

---

### 🛠️ 技术栈 (Tech Stack)

<div align="center">
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/MuJoCo-DE0027?style=for-the-badge&logo=googlecolab" alt="MuJoCo">
  <img src="https://img.shields.io/badge/PySerial-A4161A?style=for-the-badge" alt="pyserial">
  <img src="https://img.shields.io/badge/Stable%20Baselines3-4285F4?style=for-the-badge" alt="Stable Baselines3">
  <img src="https://img.shields.io/badge/Huawei%20Ascend-DE0027?style=for-the-badge&logo=huawei" alt="Huawei Ascend">
</div>

---

### 📦 安装指南 (Installation)

<details>
<summary><strong>► 点击查看详细的系统要求和安装步骤</strong></summary>

#### **系统要求**
* Python 3.7+
* `mujoco`
* `stable-baselines3[extra]`
* `pyserial`

<br>

#### **安装步骤**
1.  **克隆仓库** <br> 克隆或下载此项目到您的本地计算机。
    ```bash
    git clone [https://github.com/your-username/your-repo.git](https://github.com/your-username/your-repo.git)
    ```
2.  **进入目录** <br> 打开终端，导航到项目根目录。
    ```bash
    cd your-repo
    ```
3.  **安装依赖** <br> 运行以下命令进行安装。
    ```bash
    pip install -r requirements.txt
    ```

</details>

---

### 🚀 快速开始 (Quick Start)

<details>
<summary><strong>► 点击查看如何快速运行您的倒立摆硬件</strong></summary>

1.  **⚡️ 修改配置文件** <br> 打开 `examples/demo.py` 文件。

2.  **🔌 配置端口** <br> 根据您的操作系统和硬件连接，修改 `ENCODER_SERIAL_PORT` 和 `MOTOR_SERIAL_PORT` 的值。
    * **Windows:** `COMx` (例如: `COM3`)
    * **Linux:** `/dev/ttyUSBx` (例如: `/dev/ttyUSB0`)
    * **macOS:** `/dev/cu.usbserial-xxxx`

3.  **🔧 调整PID参数** <br> 您可以通过 `CONTROL_CONFIG` 字典来修改PID增益(`kp`, `ki`, `kd`)，以获得最佳性能。

4.  **▶️ 运行演示** <br> 在终端中，导航到 `examples/` 目录并运行：
    ```bash
    python demo.py
    ```
    系统将开始运行，您可在终端看到倒立摆的实时状态数据。按下 `Ctrl+C` 可以安全地终止程序并保存数据。

</details>

---

### 🤖 MuJoCo 仿真 (Simulation)

除了控制物理硬件，本项目还提供了基于 `MuJoCo` 的仿真环境，方便在没有硬件的情况下进行算法开发和测试。

<details>
<summary><strong>► 点击查看仿真环境的运行方法</strong></summary>

#### **1. 传统控制 (PID)**
我们实现了一个基于状态机的经典控制器，该控制器分为三个阶段，以实现从静止到平衡的全过程控制。可以直接运行 `pid_control.py` 来观察效果。
```bash
python pid_control.py
```
**控制策略详解:**
* **起摆阶段 (`SWINGUP`)**: 此阶段的目标是将摆杆从下方稳定位置摆动到顶部区域。我们采用**能量泵浦 (Energy Pumping)** 的方法：通过计算系统当前能量与目标能量（即摆杆直立时的势能）的差值，并结合摆杆的角速度和角度信息，施加一个合适的力来“注入”能量，直到摆杆到达预设的平衡角度阈值内。
* **过渡阶段 (`TRANSITION`)**: 当摆杆进入顶部平衡区域后，为了防止因速度过快而直接甩过平衡点，系统会进入一个短暂的过渡阶段。在此阶段，控制器会施加一个与角速度方向相反的阻尼力，对摆杆进行**“刹车”**，使其速度迅速降低。
* **稳定阶段 (`BALANCE`)**: 在过渡阶段结束后，系统切换到最终的平衡控制模式。此模式采用**级联PID (Cascaded PID)** 控制：
    * **角度环 (内环)**: 一个PID控制器负责根据当前摆杆角度与垂直目标点的误差，计算出维持平衡所需的基本力。
    * **位置环 (外环)**: 另一个PID控制器负责根据滑块当前位置与中心目标点的误差，计算出使滑块回归中心的修正力。
    * 最终施加到滑块上的力是这两个PID控制器输出的合力，从而同时实现摆杆的平衡和滑块的居中。

#### **2. 强化学习 (PPO)**
我们还提供了使用 `Stable Baselines3` 库训练的 PPO 智能体。

**训练模型:**
运行 `train.py` 脚本来开始训练。训练日志和模型文件将保存在项目目录下。
```bash
python train.py
```

**评估模型:**
训练完成后，运行 `evaluate.py` 来加载训练好的模型并在仿真环境中进行测试。
```bash
python evaluate.py
```
这将启动一个带渲染的评估过程，您可以直观地看到智能体的表现。

</details>

---

### 🎬 视频演示 (Demo)

<table>
  <tr>
    <td align="center">
      <img src="resources/pid.gif" alt="核心功能演示" width="320">
      <br><sub><b>核心PID控制演示</b></sub>
    </td>
    <td align="center">
      <img src="resources/slope.gif" alt="进阶功能演示" width="320">
      <br><sub><b>坡道平衡演示</b></sub>
    </td>
    <td align="center">
      <img src="resources/mujoco_inv.gif" alt="仿真视频展示" width="320">
      <br><sub><b>MuJoCo仿真演示</b></sub>
    </td>
  </tr>
</table>

---

### 🤝 参与贡献 (Contributing)

<table>
  <tr>
    <td width="70%" valign="top">
      <h3>欢迎加入我们！</h3>
      <p>我们热烈欢迎任何形式的贡献，无论是新功能的建议、代码优化，还是文档的改进。如果您对本项目感兴趣，并且希望使其变得更好，请不要犹豫！</p>
      <ul>
        <li>🐛 <strong>发现 Bug？</strong> 请通过 <a href="https://github.com/your-username/your-repo/issues">Issues</a> 告诉我们。</li>
        <li>✨ <strong>有新想法？</strong> 欢迎提交 <a href="https://github.com/your-username/your-repo/pulls">Pull Request</a> 与我们一同实现。</li>
        <li>📖 <strong>想了解更多？</strong> 请阅读我们的 <a href="CONTRIBUTING.md">贡献指南</a>。</li>
      </ul>
      <p>每一次贡献，都将使这个项目更加完善。期待您的加入！</p>
    </td>
    <td width="25%" align="center" valign="middle">
      <img src="https://placehold.co/200x200/DE0027/FFFFFF?text=Let's%20Build%20Together!" alt="欢迎加入我们">
    </td>
  </tr>
</table>
