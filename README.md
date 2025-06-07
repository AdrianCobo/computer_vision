# 🧠 computer_vision

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![DepthAI](https://img.shields.io/badge/DepthAI-SDK-yellowgreen)](https://docs.luxonis.com/projects/api/en/latest/)

Este repositorio contiene el código para generar nubes de puntos 3D a partir de un sistema multicámara estéreo OAK-D-Lite sincronizado, y proporciona herramientas avanzadas para la calibración geométrica entre cámaras usando el algoritmo ICP.

---

## 🎯 Funcionalidades principales

- ✅ **Generación de nube de puntos 3D del sistema completo**  
  A partir de las imágenes de disparidad generadas por cada cámara y sincronizadas en tiempo.

- 🧩 **Calibración entre cámaras mediante ICP**, con tres modos distintos:
  - Usando imágenes de disparidad (generación directa de la nube).
  - A partir de archivos `.pcd` almacenados previamente.
  - En tiempo real, suscribiéndose a los topics ROS 2 necesarios.

---

## 🗂️ Estructura del repositorio

```bash
computer_vision/
├── calibration/
│   ├── calibrate_from_disparity.cpp   # ICP usando imágenes de disparidad
│   ├── calibrate_from_pcd.cpp         # ICP usando archivos .pcd
│   ├── calibrate_from_topics.cpp      # ICP en tiempo real desde topics ROS 2
│
├── launch/
│   └── full_pipeline.launch.py        # Pipeline completo: sincronización, reconstrucción y fusión
│
├── utils/
│   └── scripts/                       # Scripts auxiliares para visualización, conversión, etc.
