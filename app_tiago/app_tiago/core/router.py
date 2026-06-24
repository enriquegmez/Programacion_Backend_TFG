"""
router.py
El Cerebro y Jefe de Sala del protocolo.
Orquesta la validación, comprueba los estados y ejecuta las acciones.
"""

import asyncio
import logging
import time
import socket
import os         # ¡NUEVO! Para leer variables de entorno
import psutil     # ¡NUEVO! Para la telemetría del PC (Acuérdate de hacer pip install psutil) 
import subprocess # ¡Añade esto arriba del todo en tus imports!
from typing import cast, Any, Optional, List
from app_tiago.utils.constants import MsgType, Action, StatusCode, RespType, Resource
from app_tiago.protocol.models import (
    RobotMessage, MessageHeader, GenericRespPayload, 
    ProtocolErrorPayload, EmptyPayload, AsyncNotifyPayload,
    ActionFeedbackPayload
)
from app_tiago.protocol.json_translator import MessageCodec
from app_tiago.protocol.models import (
    CommandReqPayload, ActionReqPayload, ControlModeReqPayload, 
    ControlReqPayload, StreamReqPayload, StopStreamReqPayload, 
    StreamRespPayload, QueryReqPayload, QueryRespPayload, StopActionReqPayload
)

class MessageRouter:

    # ¡NUEVO! Abstracción de calidades de cámara
    CAMERA_PROFILES = {
        "low": "&width=320&height=240&quality=30",
        "medium": "&width=640&height=480&quality=60",
        "high": "&width=1024&height=768&quality=90"
    }

    def __init__(self, connection_manager, state_machine, ros_node=None):
        self.logger = logging.getLogger("MessageRouter")
        self.connection_manager = connection_manager
        self.state_machine = state_machine
        self.ros_node = ros_node 
        self.codec = MessageCodec()
        
        # Temporizador para no saturar la red con ControlResp
        self.last_control_resp_time = 0.0
        self.CONTROL_RESP_INTERVAL = 0.5  # Segundos entre ACKs de movimiento
        self.last_control_req_arrival = 0.0

        # Watchdog interno para ROS 2
        self.is_monitoring = False
        self.monitor_task = None

    # Para obtener nuestra IP local
    def _get_local_ip(self) -> str:
        """Utilidad: Obtiene la IP local del servidor en la red."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"
        
    def _get_host_telemetry(self) -> dict:
        """Lee el estado del PC del robot. Si algo falla, devuelve None para ese campo."""
        # 1. CPU
        try:
            cpu_pct = psutil.cpu_percent(interval=0.1)
        except Exception as e:
            self.logger.error(f"Fallo al leer CPU: {e}")
            cpu_pct = None  # ¡Cambio!

        # 2. RAM
        try:
            mem = psutil.virtual_memory()
            ram_used_gb = round(mem.used / (1024**3), 2)
            ram_total_gb = round(mem.total / (1024**3), 2)
            ram_pct = mem.percent
        except Exception as e:
            self.logger.error(f"Fallo al leer RAM: {e}")
            ram_used_gb = None
            ram_total_gb = None
            ram_pct = None

        # 3. Temperatura
        temp_c = None # Empezamos asumiendo que es nulo
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for name, entries in temps.items():
                    if entries:
                        temp_c = entries[0].current
                        break
        except Exception as e:
            self.logger.warning(f"Fallo al leer Sensores de Temperatura: {e}")

        # 4. Red ROS 2
        try:
            ros_distro = os.environ.get('ROS_DISTRO', None)
            domain_id = os.environ.get('ROS_DOMAIN_ID', None)
            current_dds = os.environ.get('RMW_IMPLEMENTATION', None)

            discovery_server = os.environ.get('ROS_DISCOVERY_SERVER', '')
            use_discovery = True if discovery_server else False

            # ¡LA SOLUCIÓN! Declaramos explícitamente que es una lista de strings o nulo
            rmws: Optional[List[str]] = None
            if ros_distro:
                base_path = f"/opt/ros/{ros_distro}/share"
                if os.path.exists(base_path):
                    rmws = []
                    for folder in os.listdir(base_path):
                        if folder.startswith('rmw_') and folder.endswith('_cpp') and "implementation" not in folder:
                            rmws.append(folder)
                    if not rmws:
                        rmws = None

        except Exception as e:
            self.logger.error(f"Fallo leyendo el entorno ROS: {e}")
            ros_distro = None
            domain_id = None
            current_dds = None
            rmws = None
            use_discovery = None

        return {
            "cpu_pct": cpu_pct,
            "ram_used_gb": ram_used_gb,
            "ram_total_gb": ram_total_gb,
            "ram_pct": ram_pct,
            "temp_c": temp_c,
            "ros_distro": ros_distro,
            "ros_domain_id": domain_id,
            "current_dds": current_dds,
            "available_dds": rmws,
            "use_discovery": use_discovery
        }

    def _write_env_file(self, domain_id: str, dds: str, use_discovery: bool) -> bool:
        """Sobrescribe el archivo maestro y gestiona el Discovery Server nativo de Linux."""
        try:
            ros_distro = os.environ.get('ROS_DISTRO', 'humble') # Por defecto humble o el que tengas

            # 1. Escribimos nuestro archivo de configuración
            env_path = os.path.expanduser('~/.tiago_app_config.env')
            with open(env_path, 'w') as f:
                f.write(f"export ROS_DOMAIN_ID={domain_id}\n")
                f.write(f"export RMW_IMPLEMENTATION={dds}\n")
                # Si el usuario lo pide y usamos FastDDS, inyectamos la variable
                if use_discovery and "fastrtps" in dds.lower():
                    f.write(f"export ROS_DISCOVERY_SERVER=127.0.0.1:11811\n")
                    
            # 2. Automatizamos el .bashrc de forma segura (como ya teníamos)
            bashrc_path = os.path.expanduser('~/.bashrc')
            source_line = "source ~/.tiago_app_config.env"
            
            line_exists = False
            if os.path.exists(bashrc_path):
                with open(bashrc_path, 'r') as f:
                    if source_line in f.read():
                        line_exists = True
            
            if not line_exists:
                with open(bashrc_path, 'a') as f:
                    f.write("\n# Añadido automáticamente por TIAGo App (Configuración de Red)\n")
                    f.write(source_line + "\n")

            # ==========================================
            # 3. ¡LA MAGIA! EL SERVICIO SYSTEMD DE LINUX
            # ==========================================
            service_dir = os.path.expanduser('~/.config/systemd/user')
            os.makedirs(service_dir, exist_ok=True)
            service_path = os.path.join(service_dir, 'fastdds-discovery.service')

            if use_discovery and "fastrtps" in dds.lower():
                # Creamos el archivo del servicio para que Linux lo arranque al encender el robot
                service_content = f"""[Unit]
Description=Fast DDS Discovery Server for TIAGo App
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

                # Le decimos a Linux que recargue sus archivos y active el nuestro sin sudo
                subprocess.run(["systemctl", "--user", "daemon-reload"], check=False)
                subprocess.run(["systemctl", "--user", "enable", "fastdds-discovery.service"], check=False)
                subprocess.run(["systemctl", "--user", "start", "fastdds-discovery.service"], check=False)
                self.logger.info("✅ Discovery Server configurado como servicio nativo de Linux.")
            else:
                # Si desmarcamos la casilla, destruimos el servicio y lo paramos
                if os.path.exists(service_path):
                    subprocess.run(["systemctl", "--user", "stop", "fastdds-discovery.service"], check=False)
                    subprocess.run(["systemctl", "--user", "disable", "fastdds-discovery.service"], check=False)
                    os.remove(service_path)
                    subprocess.run(["systemctl", "--user", "daemon-reload"], check=False)
                    self.logger.info("❌ Discovery Server desactivado.")

            return True

        except Exception as e:
            self.logger.error(f"❌ Error configurando el entorno o el .bashrc: {e}")
            return False

    async def _execute_power_command(self, action: str):
        """Usa DBus para mandar la orden al SO y captura errores para depuración."""
        await asyncio.sleep(1.0)
        self.logger.critical(f"¡EJECUTANDO COMANDO DE ENERGÍA: {action.upper()}!")
        
        try:
            # Seleccionamos el comando exacto
            dbus_method = "Reboot" if action == Action.REBOOT else "PowerOff"
            cmd = [
                "dbus-send", "--system", "--print-reply", 
                "--dest=org.freedesktop.login1", "/org/freedesktop/login1", 
                f"org.freedesktop.login1.Manager.{dbus_method}", "boolean:true"
            ]
            
            # Ejecutamos el comando y capturamos la salida de Linux
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                # ¡AQUÍ ESTÁ EL CHIVATO! Si Linux rechaza el apagado, lo verás en la consola
                self.logger.error(f"❌ FALLO CRÍTICO DE ENERGÍA: Linux rechazó el comando.")
                self.logger.error(f"Motivo (stderr): {result.stderr.strip()}")
            else:
                self.logger.info("✅ Comando de energía aceptado por el Sistema Operativo.")
                
        except Exception as e:
            self.logger.error(f"❌ Excepción interna al intentar ejecutar comando de energía: {e}")
    # ==========================================
    # EL WATCHDOG (Perro Guardián ROS 2)
    # ==========================================
    async def _monitor_robot_connection(self, send_callback, session_id):
        self.logger.info("Watchdog iniciado: Vigilando conexión y unicidad del robot...")
        
        while self.is_monitoring:
            await asyncio.sleep(2.0)
            
            if self.ros_node:
                status = self.ros_node.check_connection()
                
                # Si es 0 (Desconectado) o 2 (Múltiples robots en red)
                if status == 0 or status == 2:
                    self.is_monitoring = False
                    self.ros_node.disconnect_from_robot()
                    
                    # Decidimos el mensaje exacto para el móvil
                    if status == 2:
                        self.logger.error("¡Watchdog detecta MÚLTIPLES ROBOTS en la red! Abortando teleoperación por seguridad.")
                        error_detail = "MULTIPLE_ROBOTS_DETECTED"
                    else:
                        self.logger.error("¡Watchdog detecta pérdida de conexión con el robot!")
                        error_detail = "ROBOT_CONNECTION_LOST"
                        
                    notify_header = MessageHeader(
                        msg_id=0,
                        type=MsgType.ASYNC_NOTIFY,
                        session_id=session_id
                    )
                    
                    notify_payload = AsyncNotifyPayload(
                        type="EMERGENCY_STOP", 
                        details=error_detail,
                        severity="CRITICAL"
                    )
                    
                    msg = RobotMessage(header=notify_header, payload=notify_payload)
                    
                    self.state_machine.trigger_session_reset()
                    await self._send_msg(msg, send_callback)
                    break

    async def send_session_assigned(self, session_id: str, send_callback):
        header = MessageHeader(msg_id=0, type=MsgType.ASYNC_NOTIFY, session_id=session_id)
        payload = AsyncNotifyPayload(type="session_id", details=f"SESSION_ASSIGNED:{session_id}")
        await self._send_msg(RobotMessage(header=header, payload=payload), send_callback)

    # ==========================================
    # ENTRADA PRINCIPAL DESDE EL SERVIDOR
    # ==========================================
    async def handle_raw_message(self, raw_string: str, send_callback, close_callback):
        # 1. ADUANA
        msg = self.codec.decode(raw_string)
        
        if msg.header.type == MsgType.PROTOCOL_ERROR:
            if msg.header.msg_id == -1:
                await self._send_msg(msg, send_callback)
            else:
                self.logger.error(f"El cliente reportó un error de protocolo: {msg.payload.description}")
            return

        # 2. SEGURIDAD DE SESIÓN
        if not self.connection_manager.is_valid_session(msg.header.session_id):
            self.logger.warning(f"Intento de acceso con session_id inválido: {msg.header.session_id}")
            error_msg = self._build_error_msg(StatusCode.FORBIDDEN, "Invalid session_id", msg.header.msg_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 3. SEMÁFORO (Solo validación visual)
        is_valid, code, error_desc = self.state_machine.can_transition(msg)
        if not is_valid:
            error_msg = self._build_error_msg(code, error_desc, msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)
            return

        # 4. DISTRIBUCIÓN
        try:
            await self._route_message(msg, send_callback, close_callback)
        except Exception as e:
            self.logger.error(f"Error interno procesando el mensaje: {e}")
            error_msg = self._build_error_msg(StatusCode.INTERNAL_ERROR, "Internal server error", msg.header.msg_id, msg.header.session_id)
            await self._send_msg(error_msg, send_callback)

    # ==========================================
    # ENRUTADOR INTERNO
    # ==========================================
    async def _route_message(self, msg: RobotMessage, send_callback, close_callback):
        msg_type = msg.header.type
        session_id = msg.header.session_id
        req_msg_id = msg.header.msg_id

        # Cabecera genérica (Eco del ID)
        resp_header = MessageHeader(msg_id=req_msg_id, type=MsgType.RESP, session_id=session_id)
        resp_payload: Any = None

        match msg_type:
            
            case MsgType.PING_REQ:
                self.connection_manager.notify_ping_received()
                resp_header.type = MsgType.ACK
                resp_payload = EmptyPayload()

            case MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload) 
                action = cmd_payload.action
                match action:
                    case Action.CONNECT:
                        try:
                            # Comprobamos que el gestor ROS 2 exista
                            if not self.ros_node:
                                resp_payload = GenericRespPayload(
                                    success=False, code=StatusCode.INTERNAL_ERROR, 
                                    resp_type=RespType.COMMAND_RESP, details="Backend ROS 2 no inicializado."
                                )
                            else:
                                # Llamamos a la función que ahora devuelve un int (0, 1 o 2)
                                connection_status = self.ros_node.connect_to_robot()
                                
                                if connection_status == 0:
                                    # Estado 0: No hay robot
                                    resp_payload = GenericRespPayload(
                                        success=False, code=StatusCode.NOT_FOUND, 
                                        resp_type=RespType.COMMAND_RESP, details="Robot Tiago no detectado en la red."
                                    )
                                elif connection_status == 2:
                                    # Estado 2: Conflicto / Múltiples robots
                                    # Usamos BAD_REQUEST o un código similar para indicar conflicto
                                    resp_payload = GenericRespPayload(
                                        success=False, code=StatusCode.BAD_REQUEST, 
                                        resp_type=RespType.COMMAND_RESP, details="Conflicto: Múltiples robots detectados en la misma red Wi-Fi."
                                    )
                                elif connection_status == 1:
                                    # Estado 1: Todo OK
                                    self.logger.info("Conexión exitosa con el robot.")

                                    # ==========================================
                                    # ¡NUEVO! SOLTAMOS AL PERRO GUARDIÁN (WATCHDOG)
                                    # ==========================================
                                    if not self.is_monitoring:
                                        self.is_monitoring = True
                                        # Creamos una tarea asíncrona que correrá en paralelo
                                        self.monitor_task = asyncio.create_task(
                                            self._monitor_robot_connection(send_callback, session_id)
                                        )

                                    resp_payload = GenericRespPayload(
                                        success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                                    )
                        except Exception as e:
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error interno al conectar: {str(e)}"
                            )
                    # ¡NUEVO! Comandos de Lobby (Configuración de Red)
                    case Action.CHANGE_VAR:
                        domain_id = cmd_payload.param1 or "0"
                        dds = cmd_payload.param2 or "rmw_fastrtps_cpp"
                        use_discovery = cmd_payload.param3 if cmd_payload.param3 is not None else False
                        
                        success = self._write_env_file(domain_id, dds, use_discovery)
                        if success:
                            resp_payload = GenericRespPayload(success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP)
                        else:
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details="Error escribiendo configuración .env"
                            )

                    # ¡NUEVO! Comandos de Lobby (Energía)
                    case Action.REBOOT | Action.SHUTDOWN:
                        resp_payload = GenericRespPayload(success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP)
                        # Lanzamos la orden mortal como una tarea asíncrona para que no bloquee este return
                        asyncio.create_task(self._execute_power_command(action))                    
                    case Action.DISCONNECT:
                        try:

                            # ==========================================
                            # ¡NUEVO! DORMIMOS AL GUARDIÁN
                            # ==========================================
                            self.is_monitoring = False
                            if self.monitor_task:
                                self.monitor_task.cancel() # Forzamos que la tarea en segundo plano muera

                            if self.ros_node:
                                self.ros_node.stop_robot()
                                self.ros_node.disconnect_from_robot()
                                
                            # Si llega aquí, es que no ha habido errores
                            resp_payload = GenericRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                            )
                        except Exception as e:
                            # Creamos el payload de error DENTRO del except, donde 'e' todavía existe
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, 
                                resp_type=RespType.COMMAND_RESP, details=f"Error al desconectar: {str(e)}"
                            )

                    case Action.END:

                        # ==========================================
                        # ¡NUEVO! DORMIMOS AL GUARDIÁN POR SEGURIDAD
                        # ==========================================
                        self.is_monitoring = False
                        if self.monitor_task:
                            self.monitor_task.cancel()
                            
                        # Cortar conexión es seguro, no suele fallar a nivel lógico
                        resp_payload = GenericRespPayload(
                            success=True, code=StatusCode.OK, resp_type=RespType.COMMAND_RESP
                        )

            case MsgType.QUERY_REQ:
                query_payload = cast(QueryReqPayload, msg.payload)
                
                # ¡NUEVO! Consulta de la Sala de Espera (No necesita ROS 2)
                if query_payload.resource_type == Resource.HOST_INFO:
                    telemetry_data = self._get_host_telemetry()
                    resp_payload = QueryRespPayload(
                        success=True, code=StatusCode.OK, 
                        resp_type=RespType.QUERY_RESP, data=telemetry_data
                    )
                
                elif not self.ros_node:
                    resp_payload = QueryRespPayload(
                        success=False, 
                        code=StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.QUERY_RESP, 
                        details="ROS 2 no disponible."
                    )
                
                # --- Lógica de selección de recurso ---
                
               # elif query_payload.resource_type == Resource.TELEOP:
                  #  resp_data = self.ros_node.get_teleop_topics()
                 #   resp_payload = QueryRespPayload(
                    #    success=True, 
                   #     code=StatusCode.OK, 
                  #      resp_type=RespType.QUERY_RESP, 
                 #       data=resp_data
                #    )
                
               # elif query_payload.resource_type == Resource.CAMERAS:
                 #   resp_data = self.ros_node.get_camera_topics()
                 #   resp_payload = QueryRespPayload(
                #        success=True, 
               #         code=StatusCode.OK, 
              #          resp_type=RespType.QUERY_RESP, 
             #           data=resp_data
            #        )
                
                elif query_payload.resource_type == Resource.MOVEMENTS:
                    if self.ros_node:
                        success, action_list_or_error = self.ros_node.get_available_actions()
                        if success:
                            resp_payload = QueryRespPayload(
                                success=True,
                                code=StatusCode.OK,
                                resp_type=RespType.QUERY_RESP,
                                data=action_list_or_error
                            )
                        else:
                            resp_payload = QueryRespPayload(
                                success=False,
                                code=StatusCode.INTERNAL_ERROR,
                                resp_type=RespType.QUERY_RESP,
                                details=str(action_list_or_error)
                            )
                    else:
                        resp_payload = QueryRespPayload(
                            success=False,
                            code=StatusCode.INTERNAL_ERROR,
                            resp_type=RespType.QUERY_RESP,
                            details="Backend ROS 2 no disponible."
                        )
                
                elif query_payload.resource_type == Resource.TOPICS:
                    if self.ros_node:
                        data = self.ros_node.get_all_topics()
                        if data:  # Si el diccionario tiene elementos
                            resp_payload = QueryRespPayload(
                                success=True, code=StatusCode.OK, 
                                resp_type=RespType.QUERY_RESP, data=data
                            )
                        else:     # Si está vacío
                            resp_payload = QueryRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, 
                                resp_type=RespType.QUERY_RESP, details="No se encontraron topics en la red de ROS 2."
                            )
                    else:
                        resp_payload = QueryRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.QUERY_RESP, details="Backend ROS 2 no disponible."
                        )
                
                elif query_payload.resource_type == Resource.SERVICES:
                    if self.ros_node:
                        data = self.ros_node.get_all_services()
                        if data:
                            resp_payload = QueryRespPayload(
                                success=True, code=StatusCode.OK, 
                                resp_type=RespType.QUERY_RESP, data=data
                            )
                        else:
                            resp_payload = QueryRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, 
                                resp_type=RespType.QUERY_RESP, details="No se encontraron servicios en la red de ROS 2."
                            )
                    else:
                        resp_payload = QueryRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.QUERY_RESP, details="Backend ROS 2 no disponible."
                        )

                elif query_payload.resource_type == Resource.ACTIONS:
                    if self.ros_node:
                        data = self.ros_node.get_all_actions()
                        if data:
                            resp_payload = QueryRespPayload(
                                success=True, code=StatusCode.OK, 
                                resp_type=RespType.QUERY_RESP, data=data
                            )
                        else:
                            resp_payload = QueryRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, 
                                resp_type=RespType.QUERY_RESP, details="No se encontraron acciones (Action Servers) en la red de ROS 2."
                            )
                    else:
                        resp_payload = QueryRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.QUERY_RESP, details="Backend ROS 2 no disponible."
                        )
                
                elif query_payload.resource_type == Resource.SENSORS:
                    if self.ros_node:
                        data = self.ros_node.get_available_sensors()
                        if data:
                            resp_payload = QueryRespPayload(
                                success=True, code=StatusCode.OK, 
                                resp_type=RespType.QUERY_RESP, data=data
                            )
                        else:
                            resp_payload = QueryRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, 
                                resp_type=RespType.QUERY_RESP, details="No se detectó ningún sensor compatible en la red."
                            )
                    else:
                        resp_payload = QueryRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.QUERY_RESP, details="Backend ROS 2 no disponible."
                        )
                
                elif query_payload.resource_type == Resource.ROBOT_INFO:
                    # Esta llamada ahora recopila TODA la información, 
                    # incluyendo teleop_topics y camera_topics
                    resp_data = self.ros_node.get_robot_capabilities()
                    resp_payload = QueryRespPayload(
                        success=True, 
                        code=StatusCode.OK, 
                        resp_type=RespType.QUERY_RESP, 
                        data=resp_data
                    )
                
                else:
                    resp_payload = QueryRespPayload(
                        success=False, 
                        code=StatusCode.NOT_ALLOWED, 
                        resp_type=RespType.QUERY_RESP, 
                        details=f"Recurso '{query_payload.resource_type}' desconocido o no soportado."
                    )
            
            # ==========================================
            # EL CORAZÓN DE LAS ACCIONES / MOVIMIENTOS
            # ==========================================
            case MsgType.ACTION_REQ:
                action_payload = cast(ActionReqPayload, msg.payload)
                if self.ros_node:
                    # 1. Atrapamos el "Event Loop" del servidor para poder mandarle cosas desde ROS 2
                    main_loop = asyncio.get_running_loop()

                    # 2. Definimos una función envoltorio para sincronizar estados de forma segura
                    async def _sync_and_send_feedback(fb_msg: RobotMessage):
                        self.state_machine.commit_transition(msg, fb_msg)
                        await self._send_msg(fb_msg, send_callback)

                    # 3. Este es el callback que inyectaremos en el nodo de ROS 2
                    # 3. Este es el callback que inyectaremos en el nodo de ROS 2
                    def ros2_feedback_handler(is_success: bool, done_exec: bool, progress: int, status: str):
                        # ¡Ya no adivinamos leyendo el texto! Usamos las variables directas de ROS 2
                        fb_payload = ActionFeedbackPayload(
                            success=is_success,
                            code=StatusCode.OK if is_success else StatusCode.INTERNAL_ERROR,
                            resp_type=RespType.ACTION_FEEDBACK,
                            details=status,
                            status="completed" if (done_exec and is_success) else ("failed" if not is_success else "running"),
                            progress=progress,
                            done_exec=done_exec
                        )
                        fb_msg = RobotMessage(header=resp_header, payload=fb_payload)
                        
                        # Ejecutamos el envío en el hilo principal de WebSockets
                        asyncio.run_coroutine_threadsafe(_sync_and_send_feedback(fb_msg), main_loop)

                    # 4. Inyectamos la función y lanzamos la acción
                    self.ros_node.set_action_feedback_callback(ros2_feedback_handler)
                    success, msg_str = self.ros_node.execute_action(action_payload.type, action_payload.target)
                    
                    # 5. La respuesta INICIAL (El "Vale, me pongo a ello")
                    if success:
                        resp_payload = ActionFeedbackPayload(
                            success=True,
                            code=StatusCode.OK,
                            resp_type=RespType.ACTION_FEEDBACK,
                            details=msg_str,
                            status="accepted",
                            progress=0,
                            done_exec=False
                        )
                    else:
                        resp_payload = ActionFeedbackPayload(
                            success=False,
                            code=StatusCode.NOT_ALLOWED,
                            resp_type=RespType.ACTION_FEEDBACK,
                            details=msg_str,
                            status="rejected",
                            progress=0,
                            done_exec=True
                        )
                else:
                    resp_payload = ActionFeedbackPayload(
                        success=False,
                        code=StatusCode.INTERNAL_ERROR,
                        resp_type=RespType.ACTION_FEEDBACK,
                        details="Backend ROS 2 no disponible.",
                        status="error",
                        done_exec=True
                    )

            case MsgType.STOP_ACTION_REQ:
                # ¡CORRECCIÓN APLICADA! Usamos el nuevo StopActionReqPayload
                stop_action_payload = cast(StopActionReqPayload, msg.payload)
                if self.ros_node:
                    stop_success = self.ros_node.stop_action(stop_action_payload.type, stop_action_payload.target)
                    if stop_success:
                        resp_payload = GenericRespPayload(
                            success=True, 
                            code=StatusCode.OK, 
                            resp_type=RespType.STOP_ACTION_FEEDBACK, 
                            details=f"Acción '{stop_action_payload.type}' detenida correctamente.",
                        )
                    else:
                        resp_payload = GenericRespPayload(
                            success=False, 
                            code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.STOP_ACTION_FEEDBACK, 
                            details=f"Acción '{stop_action_payload.type}' no se pudo detener.",
                        )  
                else:
                    resp_payload = GenericRespPayload(
                            success=False, 
                            code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.STOP_ACTION_FEEDBACK, 
                            details=f"Backend ROS 2 no disponible, no se pudo detener la acción '{stop_action_payload.type}'.",
                        )

            case MsgType.CONTROL_MODE_REQ:
                # SOLUCIÓN: Usamos cast para que Mypy sepa qué Payload es
                ctrl_mode_payload = cast(ControlModeReqPayload, msg.payload)
                event = ctrl_mode_payload.event
                control_type = ctrl_mode_payload.type # Pillamos el type (JOINT o TELEOP)
                # Usamos el topic relativo (sin barra) como vimos antes
                # Extraemos el valor que venga. Si es None o una cadena vacía "", usará 'cmd_vel'
                raw_topic = getattr(msg.payload, 'topic', None)
                topic = raw_topic if raw_topic else "cmd_vel"
                
                try:
                    if self.ros_node:
                        # Extraemos la tupla con el resultado y el mensaje de error
                        success, error_msg = self.ros_node.set_control_mode(event, control_type, topic)
                        self.last_control_req_arrival = 0.0  # Reset del watchdog de intervalo
                        
                        if not success:
                            # 1. Topic no válido o robot desconectado
                            resp_payload = GenericRespPayload(
                                success=False, code=StatusCode.BAD_REQUEST, 
                                resp_type=RespType.CONTROL_MODE_RESP, details=error_msg
                            )
                        else:
                            # 2. Todo OK, empezamos a publicar
                            resp_payload = GenericRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.CONTROL_MODE_RESP
                            )
                    else:
                        # Falla porque el gestor ROS 2 no está instanciado
                        resp_payload = GenericRespPayload(
                            success=False, code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_MODE_RESP, details="Backend ROS 2 no disponible."
                        )
                        
                except Exception as err:
                    self.logger.error(f"Excepción en CONTROL_MODE_REQ: {err}")
                    resp_payload = GenericRespPayload(
                        success=False, code=StatusCode.INTERNAL_ERROR, 
                        resp_type=RespType.CONTROL_MODE_RESP, details=f"Excepción en el hardware: {str(err)}"
                    )

            case MsgType.CONTROL_REQ:
                ctrl_payload = cast(ControlReqPayload, msg.payload)
                v = ctrl_payload.data.v
                w = ctrl_payload.data.w
                
                joint_name = ctrl_payload.data.joint_name
                joint_value = ctrl_payload.data.joint_value
                
                current_time = time.time()
                publish_success = True
                publish_error_msg = "" # ¡NUEVO! Motivo del fallo
                
                is_teleop = not joint_name 

                # ==========================================
                # 1. LÓGICA DE RUEDAS (Con Watchdog y Escudo)
                # ==========================================
                if is_teleop:
                    if self.last_control_req_arrival > 0:
                        time_since_last_packet = current_time - self.last_control_req_arrival
                    else:
                        time_since_last_packet = 0.0

                    if time_since_last_packet > 0.6:
                        self.logger.warning(f"⚠️ HUECO DE RED: {time_since_last_packet:.2f}s sin recibir órdenes.")
                        publish_success = False
                        publish_error_msg = f"Red inestable (Salto de {time_since_last_packet:.2f}s)."
                        if self.ros_node: self.ros_node.stop_robot()
                    else:
                        if self.ros_node:
                            try:
                                # ¡AQUÍ ESTÁ LA LECTURA DE LA TUPLA!
                                publish_success, publish_error_msg = self.ros_node.publish_velocity(v, w)
                            except Exception as e:
                                publish_success = False
                                publish_error_msg = "Error interno publicando velocidad."
                        else:
                            publish_success = False
                            publish_error_msg = "Backend ROS 2 no disponible."

                    self.last_control_req_arrival = current_time

                # ==========================================
                # 2. LÓGICA DE ARTICULACIONES
                # ==========================================
                else:
                    if self.ros_node:
                        try:
                            publish_success = self.ros_node.publish_joint_position(joint_name, joint_value)
                            if not publish_success: publish_error_msg = "Error en articulación."
                        except Exception:
                            publish_success = False
                            publish_error_msg = "Excepción en articulación."
                    else:
                        publish_success = False
                        publish_error_msg = "Backend ROS 2 no disponible."
                    
                    self.last_control_req_arrival = current_time

                # ==========================================
                # 3. GESTIÓN DE RESPUESTAS AL MÓVIL
                # ==========================================
                if not publish_success or (current_time - self.last_control_resp_time >= self.CONTROL_RESP_INTERVAL):
                    self.last_control_resp_time = current_time
                    
                    if not publish_success:
                        resp_payload = GenericRespPayload(
                            success=False, 
                            code=StatusCode.INTERNAL_ERROR, 
                            resp_type=RespType.CONTROL_RESP, 
                            details=publish_error_msg # MANDAMOS EL TEXTO EXACTO AL MÓVIL
                        )
                    else:
                        resp_payload = GenericRespPayload(
                            success=True, 
                            code=StatusCode.OK, 
                            resp_type=RespType.CONTROL_RESP
                        )

            # ==========================================
            # NUEVO BLOQUE: MONITORIZACIÓN / VÍDEO
            # ==========================================
            case MsgType.STREAM_REQ:
                stream_payload = cast(StreamReqPayload, msg.payload)
                
                if stream_payload.resource == "camera":
                    topic = stream_payload.topic
                    if not topic:
                        resp_payload = StreamRespPayload(
                            success=False, code=StatusCode.BAD_REQUEST, resp_type=RespType.STREAM_RESP,
                            details="El topic de la cámara es obligatorio."
                        )
                    else:
                        # --- 1. COMPROBAR QUE EL SERVIDOR WEB DE VÍDEO FUNCIONA ---
                        if self.ros_node and not self.ros_node.is_video_server_running():
                            self.logger.warning("Petición de vídeo rechazada: web_video_server no está corriendo.")
                            resp_payload = StreamRespPayload(
                                success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.STREAM_RESP,
                                details="El servidor de vídeo del robot está apagado o no responde."
                            )
                        # --- 2. ¡NUEVA COMPROBACIÓN! EL TOPIC EXISTE EN ROS 2 ---
                        elif self.ros_node and not self.ros_node.is_topic_active(topic):
                            self.logger.warning(f"Petición de vídeo rechazada: El topic {topic} no existe en la red.")
                            resp_payload = StreamRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, resp_type=RespType.STREAM_RESP,
                                details=f"La cámara está desconectada o el topic '{topic}' es incorrecto."
                            )
                        else:
                            # --- 3. TODO OK: CREAR URL ---
                            server_ip = self._get_local_ip()
                            quality = stream_payload.quality_level if stream_payload.quality_level else "medium"
                            url_params = self.CAMERA_PROFILES.get(quality, self.CAMERA_PROFILES["medium"])

                            #server_ip = "192.168.68.88"
                            
                            final_url = f"http://{server_ip}:8081/stream?topic={topic}{url_params}"
                            
                            self.logger.info(f"Stream de cámara solicitado. Asignando URL: {final_url}")
                            resp_payload = StreamRespPayload(
                                success=True, code=StatusCode.OK, resp_type=RespType.STREAM_RESP, stream_url=final_url
                            )
                elif stream_payload.resource.upper() == Resource.SENSORS:
                    topic = stream_payload.topic
                    if not topic:
                        resp_payload = StreamRespPayload(
                            success=False, code=StatusCode.BAD_REQUEST, resp_type=RespType.STREAM_RESP,
                            details="El topic del sensor es obligatorio."
                        )
                    elif self.ros_node:
                        if not self.ros_node.is_topic_active(topic):
                            resp_payload = StreamRespPayload(
                                success=False, code=StatusCode.NOT_FOUND, resp_type=RespType.STREAM_RESP,
                                details=f"El sensor en '{topic}' no existe o está apagado."
                            )
                        else:
                            # 1. Atrapamos el hilo asíncrono para enviar respuestas de forma continua
                            main_loop = asyncio.get_running_loop()

                            async def _send_sensor_data(sensor_json: dict):
                                # Cada vez que haya un dato nuevo, lo enviamos al móvil en stream_data
                                data_payload = StreamRespPayload(
                                    success=True, code=StatusCode.OK, resp_type=RespType.STREAM_RESP,
                                    stream_data=sensor_json
                                )
                                msg_out = RobotMessage(header=resp_header, payload=data_payload)
                                await self._send_msg(msg_out, send_callback)
                                
                            def ros2_sensor_callback(sensor_json: dict):
                                # Función puente: ROS 2 llama a esto, y esto envía por WebSocket
                                asyncio.run_coroutine_threadsafe(_send_sensor_data(sensor_json), main_loop)

                            # 2. Le pedimos a ROS 2 que se suscriba al topic y empiece a mandar datos
                            success = self.ros_node.start_sensor_stream(topic, ros2_sensor_callback)
                            
                            # 3. Respuesta INICIAL (Avisamos al móvil de que el grifo se ha abierto)
                            if success:
                                resp_payload = StreamRespPayload(
                                    success=True, code=StatusCode.OK, resp_type=RespType.STREAM_RESP,
                                    details=f"Stream de sensor '{topic}' iniciado."
                                )
                            else:
                                resp_payload = StreamRespPayload(
                                    success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.STREAM_RESP,
                                    details=f"No se pudo iniciar el stream del sensor '{topic}'."
                                )
                    else:
                         resp_payload = StreamRespPayload(
                             success=False, code=StatusCode.INTERNAL_ERROR, resp_type=RespType.STREAM_RESP,
                             details="Backend ROS 2 no disponible."
                         )
                else:
                    resp_payload = StreamRespPayload(
                        success=False, code=StatusCode.NOT_ALLOWED, resp_type=RespType.STREAM_RESP,
                        details=f"El recurso '{stream_payload.resource}' aún no está implementado."
                    )
            #No miramos caso de si el servidor esta apagado para devolver error porque el móvil no tiene por qué saberlo. 
            # Si el stream no funciona, el móvil lo detectará al no recibir datos y podrá mostrar un mensaje genérico de 
            # "No se puede mostrar el vídeo".
            case MsgType.STOP_STREAM_REQ:
                stop_stream_payload = cast(StopStreamReqPayload, msg.payload)
                
                # Leemos el topic de forma segura por si viene None
                topic = getattr(stop_stream_payload, 'topic', 'Desconocido')
                self.logger.info(f"Petición de parada de stream: {stop_stream_payload.resource} - {topic}")
                
                if stop_stream_payload.resource.upper() == Resource.SENSORS and self.ros_node:
                    if topic != 'Desconocido':
                        self.ros_node.stop_sensor_stream(topic)
                
                resp_payload = GenericRespPayload(
                    success=True, code=StatusCode.OK, resp_type=RespType.STOP_STREAM_RESP
                )

            case MsgType.ACK:
                self.logger.debug(f"Recibido ACK del cliente para el msg_id: {req_msg_id}")

            case _:
                self.logger.warning(f"Mensaje {msg_type} rutado pero no implementado.")

        # ==========================================
        # EL COMMIT DE ESTADO Y ENVÍO FINAL
        # ==========================================
        if resp_payload:
            resp_msg = RobotMessage(header=resp_header, payload=resp_payload)

            # --- ¡AÑADE ESTA LÍNEA AQUÍ! ---
            if msg_type == MsgType.QUERY_REQ:
                print(f"\n[CHIVATO PYTHON] Voy a enviar esto al móvil: {resp_payload}\n")
            # -------------------------------
            
            # 1. Sincronizamos la máquina de estados con lo que acaba de suceder
            self.state_machine.commit_transition(msg, resp_msg)
            
            # 2. Enviamos la respuesta por el WebSocket
            await self._send_msg(resp_msg, send_callback)
            
            # 3. Novedad: Si era END y ha ido bien, llamamos a la función del servidor
            # SOLUCIÓN: Usamos cast de nuevo y nos aseguramos de que resp_payload sea GenericRespPayload
            if msg_type == MsgType.COMMAND_REQ:
                cmd_payload = cast(CommandReqPayload, msg.payload)
                # ¡NUEVO! Añadidos REBOOT y SHUTDOWN
                if cmd_payload.action in [Action.END, Action.REBOOT, Action.SHUTDOWN]:
                    # Comprobamos el success de forma segura con getattr
                    is_success = getattr(resp_payload, 'success', False)
                    if is_success:
                        self.logger.info("Cerrando el túnel WebSocket de forma controlada...")
                        await close_callback()

    # ==========================================
    # UTILIDADES
    # ==========================================
    async def _send_msg(self, msg: RobotMessage, send_callback):
        json_str = self.codec.encode(msg)
        await send_callback(json_str)

    def _build_error_msg(self, code: int, description: str, req_msg_id: int, session_id: str = "") -> RobotMessage:
        header = MessageHeader(msg_id=req_msg_id, type=MsgType.PROTOCOL_ERROR, session_id=session_id)
        payload = ProtocolErrorPayload(error_code=code, description=description)
        return RobotMessage(header=header, payload=payload)