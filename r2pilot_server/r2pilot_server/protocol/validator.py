## @file validator.py
#  @brief El validador del Protocolo R2Pilot.
#  @details Carga el contrato JSON Schema y valida matemáticamente la estructura 
#           de los diccionarios entrantes antes de que el servidor intente procesarlos.
#  @author Enrique Gómez
#  @date 2026

import json
import logging
from pathlib import Path
from typing import Dict, Any, Tuple

import jsonschema
from jsonschema.exceptions import ValidationError


class ProtocolValidator:
    """!
    @brief Motor de validación estricta de tramas de red.
    @details Actúa como barrera de seguridad de capa 7 (Aplicación). Garantiza que 
             todos los paquetes cumplan con las reglas de tipos, enumeraciones, límites 
             y campos obligatorios definidos en el JSON Schema oficial del protocolo.
    """

    def __init__(self) -> None:
        """!
        @brief Inicializa el validador y precarga el esquema en memoria.
        """
        self.logger = logging.getLogger("ProtocolValidator")
        self.schema = self._load_schema()

    def _load_schema(self) -> Dict[str, Any]:
        """!
        @brief Carga y procesa el archivo de contrato del protocolo en disco.
        @details Resuelve la ruta de forma relativa a este mismo script, garantizando que 
                 el archivo se encuentre independientemente de si se ejecuta en local, 
                 en un entorno virtual o dentro de un contenedor Docker.
        @return Diccionario con el JSON Schema parseado listo para el motor de validación.
        @raises FileNotFoundError Si el archivo de esquema no existe en la carpeta adyacente.
        @raises json.JSONDecodeError Si el archivo del esquema está corrupto o mal formado.
        """
        # __file__ apunta a la ruta absoluta de validator.py. .parent extrae su directorio contenedor.
        schema_path = Path(__file__).parent / "json_schema.json"
        
        try:
            with open(schema_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            self.logger.critical(f"[SISTEMA] No se encontró el contrato (JSON Schema) en: {schema_path}")
            raise
        except json.JSONDecodeError as e:
            self.logger.critical(f"[SISTEMA] El archivo del contrato (json_schema.json) está corrupto: {e}")
            raise

    def validate_message(self, message_dict: Dict[str, Any]) -> Tuple[bool, str]:
        """!
        @brief Comprueba si un diccionario entrante obedece el contrato de R2Pilot.
        @details Utiliza la librería jsonschema (Draft-07). Si detecta una infracción estructural, 
                 extrae la ruta exacta del fallo para facilitar la depuración inmediata.
        @param message_dict El paquete decodificado a diccionario nativo de Python.
        @return Tupla (is_valid, error_description). Si is_valid es True, el error devuelto es "".
        """
        try:
            # Validación matemática contra el contrato cargado en memoria
            jsonschema.validate(instance=message_dict, schema=self.schema)
            return True, ""
        except ValidationError as e:
            # Extraemos la ruta del error para que el log de la consola sea quirúrgico
            # Ejemplo visual: "payload -> data -> v"
            error_path = " -> ".join([str(p) for p in e.path])
            
            if error_path:
                error_msg = f"Infracción en el campo [{error_path}]: {e.message}"
            else:
                error_msg = e.message
                
            self.logger.warning(f"[SEGURIDAD] Paquete bloqueado en la frontera: {error_msg}")
            return False, error_msg