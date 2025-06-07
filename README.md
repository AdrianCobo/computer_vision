# 🧠 computer_vision

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
![distro](https://img.shields.io/badge/ROS2-Jazzy-blue)
[![DepthAI](https://img.shields.io/badge/DepthAI-SDK-yellowgreen)](https://docs.luxonis.com/projects/api/en/latest/)

Este repositorio contiene el código para generar nubes de puntos 3D a partir de un sistema multicámara estéreo OAK-D-Lite sincronizado, y proporciona herramientas avanzadas para la calibración geométrica entre cámaras usando el algoritmo ICP.

---

## 🛠️ Instalación

Asegúrate de estar trabajando dentro de un workspace de ROS 2 (por ejemplo, `~/ros2_ws/src`):

```bash
cd ~/ros2_ws/src
git clone -b calib https://github.com/AdrianCobo/computer_vision.git
cd ..
colcon build --packages-select computer_vision
```

Una vez finalice la compilación, no olvides fuentear el entorno:
```bash
source install/setup.bash
```

## 🎯 Funcionalidades principales

- ✅ **Generación de nube de puntos 3D del sistema completo**  
  A partir de las imágenes de disparidad generadas por cada cámara y sincronizadas en tiempo.

- 🧩 **Calibración entre cámaras mediante ICP**, con tres modos distintos:
  - Usando imágenes de disparidad (generación directa de la nube).
  - A partir de archivos `.pcd` almacenados previamente.
  - En tiempo real, suscribiéndose a los topics ROS 2 necesarios.

---

## 🗂️ Estructura relevante del repositorio

```bash
computer_vision/
├── include/
│   ├── DepthSync.hpp                  # Genera la nube de puntos del sistema usando imágenes de disparidad sincronizadas
│   ├── PlcSyncIcp.hpp                # ICP usando imágenes de disparidad
│   ├── PlcSyncIcppclfrompcd.hpp      # ICP usando archivos .pcd
│   ├── PlcSyncIcppcl.hpp             # ICP en tiempo real desde topics de ROS 2
│
├── launch/                           # Launchers para facilitar el uso del código anterior
