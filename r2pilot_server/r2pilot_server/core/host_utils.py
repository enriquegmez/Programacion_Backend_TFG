## @file host_utils.py
#  @brief Caja de herramientas de bajo nivel para interactuar con el Sistema Operativo (Host OS).
#  @details Abstrae toda la lógica de control de hardware, lectura de telemetría de CPU/RAM,
#           escritura de archivos de entorno bash, configuración de demonios systemd
#           y llamadas prioritarias de energía (apagado/reinicio) de Linux.
#  @author Enrique Gómez
#  @date 2026

import logging
import os
import socket
import subprocess
import psutil  # type: ignore[import]
from typing import Optional, List, Dict, Any
from r2pilot_server.utils.constants import TelemetryKeys


class HostSystemManager:
    """!
    @brief Gestor de interacciones con el Sistema Operativo y el Hardware.
    @details Aísla las llamadas al sistema (Syscalls) y la monitorización de recursos,
             garantizando portabilidad y aislamiento de fallos.
    """

    def __init__(self) -> None:
        """!
        @brief Inicializa el gestor de recursos de sistema de bajo nivel.
        """
        self.logger = logging.getLogger("HostSystemManager")

    def get_local_ip(self) -> str:
        """!
        @brief Obtiene la dirección IPv4 local del robot en la red Wi-Fi activa.
        @details Abre un socket UDP temporal contra un servidor DNS público (sin enviar tráfico)
                 para determinar de forma segura la interfaz de red por defecto.
        @return String con la dirección IP local (ej. "192.168.1.50") o "127.0.0.1" si falla.
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            self.logger.warning("[SISTEMA] No se pudo determinar la IP local. Usando localhost por defecto.")
            return "127.0.0.1"

    def get_host_telemetry(self) -> Dict[str, Any]:
        """!
        @brief Captura el estado en tiempo real de los sensores físicos del ordenador (CPU, RAM, Temp).
        @details Diseñado con tolerancia defensiva a fallos. Si un sensor físico no responde,
                 captura la excepción y devuelve None para ese campo específico.
        @return Diccionario con las métricas de carga del hardware y variables del entorno ROS 2.
        """
        # 1. Porcentaje de carga de CPU
        try:
            cpu_pct = psutil.cpu_percent(interval=0.1)
        except Exception as e:
            self.logger.error(f"[SISTEMA] Fallo al leer sensor de CPU: {e}")
            cpu_pct = None  

        # 2. Estado de memoria RAM (Convertido a Gigabytes)
        ram_used_gb: Optional[float] = None
        ram_total_gb: Optional[float] = None
        ram_pct: Optional[float] = None
        try:
            mem = psutil.virtual_memory()
            ram_used_gb = round(mem.used / (1024**3), 2)
            ram_total_gb = round(mem.total / (1024**3), 2)
            ram_pct = mem.percent
        except Exception as e:
            self.logger.error(f"[SISTEMA] Fallo al leer sensor de RAM: {e}")
            ram_used_gb = ram_total_gb = ram_pct = None

        # 3. Lectura de temperatura física de los núcleos del procesador
        temps = psutil.sensors_temperatures()
        temp_c = None

        # Buscamos sensores comunes en arquitecturas Intel (coretemp) o AMD (k10temp)
        for chip in ("coretemp", "k10temp"):
            if chip in temps:
                for sensor in temps[chip]:
                    if sensor.label in ("Package id 0", "Tctl", "Tdie"):
                        temp_c = sensor.current
                        break
                if temp_c is None and temps[chip]:
                    temp_c = temps[chip][0].current
                break

        # Respaldo: primer sensor térmico que contenga datos
        if temp_c is None:
            for entries in temps.values():
                if entries:
                    temp_c = entries[0].current
                    break

        # 4. Inspección del entorno de red de ROS 2 del sistema operativo
        try:
            ros_distro = os.environ.get('ROS_DISTRO', None)
            domain_id = os.environ.get('ROS_DOMAIN_ID', None)
            current_dds = os.environ.get('RMW_IMPLEMENTATION', None)

            discovery_server = os.environ.get('ROS_DISCOVERY_SERVER', '')
            use_discovery = bool(discovery_server)

            # Buscamos qué middlewares (DDS) tiene instalados el Linux en sus carpetas de ROS
            rmws: Optional[List[str]] = None
            if ros_distro:
                base_path = f"/opt/ros/{ros_distro}/share"
                if os.path.exists(base_path):
                    rmws = [
                        folder for folder in os.listdir(base_path) 
                        if folder.startswith('rmw_') and folder.endswith('_cpp') and "implementation" not in folder
                    ]
                    if not rmws:
                        rmws = None
        except Exception as e:
            self.logger.error(f"[SISTEMA] Fallo leyendo el entorno de ROS 2: {e}")
            ros_distro = domain_id = current_dds = rmws = use_discovery = None

        return {
            TelemetryKeys.CPU_PCT: cpu_pct,
            TelemetryKeys.RAM_USED_GB: ram_used_gb,
            TelemetryKeys.RAM_TOTAL_GB: ram_total_gb,
            TelemetryKeys.RAM_PCT: ram_pct,
            TelemetryKeys.TEMP_C: temp_c,
            TelemetryKeys.ROS_DISTRO: ros_distro,
            TelemetryKeys.ROS_DOMAIN_ID: domain_id,
            TelemetryKeys.CURRENT_DDS: current_dds,
            TelemetryKeys.AVAILABLE_DDS: rmws,
            TelemetryKeys.USE_DISCOVERY: use_discovery
        }

    def write_env_file(self, domain_id: str, dds: str, use_discovery: bool) -> bool:
        """!
        @brief Configura dinámicamente las variables de red de ROS 2 y automatiza servicios de Linux.
        @details Escribe la configuración en un archivo .env local, inyecta su ejecución automática en
                 el .bashrc del usuario y registra/elimina el Discovery Server como demonio de systemd.
        @param domain_id El identificador numérico de subred ROS_DOMAIN_ID.
        @param dds El nombre del middleware de ROS 2 (RMW_IMPLEMENTATION).
        @param use_discovery Indica si se debe levantar de fondo el servidor de descubrimiento de FastDDS.
        @return True si las modificaciones de red se completaron, False ante errores de escritura.
        """
        try:
            ros_distro = os.environ.get('ROS_DISTRO', 'humble')

            # 1. Definimos las rutas
            env_path = os.path.expanduser('~/.R2Pilot_config.env')
            
            # RESOLUCIÓN DE RUTA: Asume que el XML está en la misma carpeta que este script de Python.
            # Ajusta 'super_client_configuration_file.xml' al nombre exacto de tu archivo.
            base_dir = os.path.dirname(os.path.abspath(__file__))
            xml_path = os.path.join(base_dir, 'super_client_configuration_file.xml')

            # 2. Escribimos nuestro archivo de configuración aislado
            with open(env_path, 'w') as f:
                f.write(f"export ROS_DOMAIN_ID={domain_id}\n")
                f.write(f"export RMW_IMPLEMENTATION={dds}\n")
                
                if use_discovery and "fastrtps" in dds.lower():
                    f.write("export ROS_DISCOVERY_SERVER=127.0.0.1:11811\n")
                    
                    # Verificamos que el archivo XML existe antes de exportarlo para evitar fallos silenciosos en ROS 2
                    if os.path.exists(xml_path):
                        f.write(f"export FASTRTPS_DEFAULT_PROFILES_FILE={xml_path}\n")
                        self.logger.info(f"[SISTEMA] Perfil Super Client inyectado desde: {xml_path}")
                    else:
                        self.logger.warning(f"[SISTEMA] No se encontró el XML de Super Client en: {xml_path}")
                else:
                    # Limpieza crítica: Si desactivan el discovery, nos aseguramos de borrar la variable
                    # para que Fast DDS no intente buscar el XML de un servidor apagado.
                    f.write("unset FASTRTPS_DEFAULT_PROFILES_FILE\n")
                    
            # 2. Aseguramos el enlace automático en el bashrc para consolas nuevas
            bashrc_path = os.path.expanduser('~/.bashrc')
            source_line = "source ~/.R2Pilot_config.env"
            
            line_exists = os.path.exists(bashrc_path) and source_line in open(bashrc_path).read()
            
            if not line_exists:
                with open(bashrc_path, 'a') as f:
                    f.write("\n# Añadido automáticamente por R2Pilot (Configuración de Red)\n")
                    f.write(source_line + "\n")

            # 3. GESTIÓN DEL SERVICIO DEMONIO DE LINUX (systemd)
            service_dir = os.path.expanduser('~/.config/systemd/user')
            os.makedirs(service_dir, exist_ok=True)
            service_path = os.path.join(service_dir, 'fastdds-discovery.service')

            if use_discovery and "fastrtps" in dds.lower():
                # Creamos el servicio systemd de usuario para automatizar el arranque en segundo plano
                service_content = f"""[Unit]
Description=Fast DDS Discovery Server for R2Pilot
After=network.target

[Service]
ExecStart=/bin/bash -c "source /opt/ros/{ros_distro}/setup.bash && fastdds discovery -i 0 -p 11811"
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
"""
                with open(service_path, 'w') as f:
                    f.write(service_content)

                # Aplicamos la recarga y levantamiento sin necesidad de usar sudo
                subprocess.run(["systemctl", "--user", "daemon-reload"], check=False)
                subprocess.run(["systemctl", "--user", "enable", "fastdds-discovery.service"], check=False)
                subprocess.run(["systemctl", "--user", "start", "fastdds-discovery.service"], check=False)
                self.logger.info("[SISTEMA] Discovery Server configurado y activado como servicio systemd de usuario.")
            else:
                # Si se desmarca, purgamos el archivo y apagamos el daemon de Linux de inmediato
                if os.path.exists(service_path):
                    subprocess.run(["systemctl", "--user", "stop", "fastdds-discovery.service"], check=False)
                    subprocess.run(["systemctl", "--user", "disable", "fastdds-discovery.service"], check=False)
                    os.remove(service_path)
                    subprocess.run(["systemctl", "--user", "daemon-reload"], check=False)
                    self.logger.info("[SISTEMA] Servicio de Discovery Server systemd desactivado y eliminado del sistema.")

            return True
        except Exception as e:
            self.logger.error(f"[SISTEMA] Error configurando el entorno o el .bashrc: {e}")
            return False

    def execute_power_command(self, action: str) -> None:
        """!
        @brief Ejecuta órdenes críticas de energía (Apagado o Reinicio) a nivel del kernel de Linux.
        @details Utiliza comandos directos del sistema de control de inicio systemctl. Requiere privilegios.
        @param action Comando de energía a ejecutar. Debe ser 'reboot' o 'shutdown'.
        @return None
        """
        self.logger.critical(f"[SISTEMA] ¡EJECUTANDO COMANDO DE ENERGÍA: {action.upper()}!")
        
        try:
            # Comando limpio de systemctl. Si falla, se captura la excepción y se loguea el error.
            cmd = ["sudo", "systemctl", "reboot"] if action == "reboot" else ["sudo", "systemctl", "poweroff"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                self.logger.error("[SISTEMA] FALLO CRÍTICO: Linux rechazó el comando de energía.")
                self.logger.error(f"Motivo del rechazo (stderr): {result.stderr.strip()}")
            else:
                self.logger.info("[SISTEMA] Petición aprobada por el kernel. Apagando equipo...")
        except Exception as e:
            self.logger.error(f"[SISTEMA] Excepción interna al intentar apagar el hardware: {e}")