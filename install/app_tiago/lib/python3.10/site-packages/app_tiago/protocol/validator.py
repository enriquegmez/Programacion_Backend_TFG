"""
validator.py
El Guardián del Protocolo.
Carga el schema.json y valida estrictamente los diccionarios entrantes
antes de que el backend intente procesarlos.
"""

import json
import logging
from pathlib import Path
import jsonschema
from jsonschema.exceptions import ValidationError

class ProtocolValidator:
    def __init__(self):
        self.logger = logging.getLogger("ProtocolValidator")
        self.schema = self._load_schema()

    def _load_schema(self) -> dict:
        """
        Carga el archivo schema.json asumiendo que está en el mismo directorio 
        que este script (validator.py). Esto evita problemas de rutas en Docker.
        """
        # __file__ es la ruta de validator.py. parent es su carpeta.
        schema_path = Path(__file__).parent / "json_schema.json"
        
        try:
            with open(schema_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            self.logger.critical(f"No se encontró el archivo de esquema en: {schema_path}")
            raise
        except json.JSONDecodeError as e:
            self.logger.critical(f"El archivo schema.json está mal formado: {e}")
            raise

    def validate_message(self, message_dict: dict) -> tuple[bool, str]:
        """
        Comprueba si un diccionario cumple estrictamente el JSON Schema cargado.
        
        Retorna:
            (True, "") si es válido.
            (False, "descripción del error") si es inválido.
        """
        try:
            jsonschema.validate(instance=message_dict, schema=self.schema)
            return True, ""
        except ValidationError as e:
            # Extraemos la ruta del error para que el log sea muy claro
            # Ejemplo: "['payload']['data']['v']"
            error_path = " -> ".join([str(p) for p in e.path])
            if error_path:
                error_msg = f"Error de protocolo en {error_path}: {e.message}"
            else:
                error_msg = e.message
                
            self.logger.warning(f"Mensaje descartado por el Validator: {error_msg}")
            return False, error_msg