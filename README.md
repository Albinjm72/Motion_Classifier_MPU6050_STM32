# 🧠 Motion Classifier with STM32G431 + MPU6050 + Edge Impulse

## 📦 Overview

This project demonstrates a real-time motion classification system on the STM32G431CBU6 using the MPU6050 accelerometer and 
an embedded machine learning model trained with Edge Impulse. Results are output via UART, with float-free formatting for compatibility
on resource-constrained embedded builds.

---

## 🧰 Prerequisites

### 🔧 Install the Edge Impulse `.pack` in STM32CubeMX

1. Open **STM32CubeMX**
2. Go to **Help → Manage embedded software packages**
3. Click the **From Local** tab
4. Select your Edge Impulse `.pack` file (downloaded from Edge Impulse Studio)
5. Click **Install**

> This package adds Edge Impulse’s signal and classifier libraries directly into your STM32 project.

---

🆕 🔨 Create STM32CubeIDE Project as C+

When starting your STM32CubeIDE project, follow these steps to ensure compatibility with Edge Impulse’s C++ SDK:
- Create a New STM32 Project in CubeIDE
- Choose your target MCU (e.g., STM32G431CBU6)
- In the "Project Setup" window:
- Under Language, select C++
- Click Finish
✅ This sets up your project with .cpp support from the beginning, avoiding build errors when integrating C++ libraries like Edge Impulse’s ei_classifier and numpy.

## 📁 Migrating from `main.c` to `main.cpp`

STM32CubeIDE typically starts with `main.c`, but this project uses C++ for Edge Impulse support. Here's what to do:

1. Rename `Core/Src/main.c` to `main.cpp`
