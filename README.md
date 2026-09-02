<div align="center">
  <!-- Si quieres que se vea el logo aquí, puedes usar la imagen subida en el frontend o borrar esta línea de img -->
  <h1>🤖 R2Pilot - Backend Server</h1>
  <p><strong>Servidor basado en ROS 2 para el control y monitorización remota de robots desde dispositivos móviles.</strong></p>
</div>

## 🎓 Contexto Académico
Este repositorio contiene el código del servidor (Backend) correspondiente al **Trabajo Fin de Grado (TFG)**:
* **Título:** *Desarrollo de una aplicación móvil para el control y monitorización de robots basados en ROS 2*
* **Autor:** Enrique Gómez Pacheco
* **Tutor:** Juan José Ramos Muñoz
* **Universidad:** Universidad de Granada (UGR) - ETSIIT / TSTC (2026)

📄 **[Consultar Memoria del TFG (PDF)](docs/Memoria_TFG_R2Pilot_Enrique_Gomez.pdf)**  
🔗 **[Ver Repositorio del Frontend (App Android)](https://github.com/enriquegmez/Programacion_Frontend_TFG.git)**  
📚 **[Ver Documentación de Código (Doxygen)](https://enriquegmez.github.io/Programacion_Backend_TFG/doxygen/html/index.html)**

---

## 📝 Descripción
Este módulo actúa como pasarela entre el ecosistema **ROS 2 (DDS)** del robot físico y la aplicación móvil Android **R2Pilot**. 


## ⚙️ Requisitos Previos
- **Sistema Operativo:** Ubuntu 22.04 LTS (o compatible).
- **Middleware:** ROS 2 Humble Hawksbill.
- **Paquetes externos (ROS 2):** 
  - `web_video_server` (para transmisión HTTP de cámaras).
  - `play_motion2` (para ejecución de acciones).

---

## 🚀 Instalación y Despliegue

**1. Preparar el entorno e instalar dependencias:**

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep
```

**2. Descargar el código en un Workspace de ROS 2:**

```bash
mkdir -p ~/r2pilot_ws/src
cd ~/r2pilot_ws/src
git clone https://github.com/enriquegmez/Programacion_Backend_TFG.git
```

**3. Instalar las dependencias del paquete y compilar:**

```bash
cd ~/r2pilot_ws

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**4. Ejecutar el servidor:**

```bash
source install/setup.bash
ros2 launch r2pilot_server backend.launch.py
```

> Por defecto, el servidor WebSocket escucha en el puerto `8765` y el `web_video_server` se inicia automáticamente.

### 🔒 Seguridad (Permisos del sistema)

Para permitir que la aplicación gestione remotamente el apagado y reinicio del robot, es necesario conceder permisos sin contraseña al usuario que ejecuta el servidor.

Ejecuta:

```bash
sudo visudo
```

y añade la siguiente línea, sustituyendo `TU_USUARIO_LINUX` por el nombre del usuario correspondiente:

```text
TU_USUARIO_LINUX ALL=(ALL) NOPASSWD: /bin/systemctl poweroff, /bin/systemctl reboot
```

