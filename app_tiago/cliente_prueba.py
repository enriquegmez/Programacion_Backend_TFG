import asyncio
import websockets
import json

async def test_connection():
    uri = "ws://127.0.0.1:8765"
    print(f"Conectando a {uri}...")
    
    async with websockets.connect(uri) as websocket:
        print("¡Conectado!")
        
        # El JSON exacto que tu servidor está esperando
        mensaje_prueba = {
            "header": {
                "msg_id": 1,
                "timestamp": 123456789.0,
                "type": "COMMAND_REQ",
                "session_id": "movil_de_prueba"
            },
            "payload": {
                "action": "connect"
            }
        }
        
        # Enviamos el mensaje
        print(f"Enviando: {mensaje_prueba}")
        await websocket.send(json.dumps(mensaje_prueba))
        
        # Esperamos la respuesta
        respuesta = await websocket.recv()
        print(f"\nRespuesta del servidor recibida:\n{respuesta}")

if __name__ == "__main__":
    asyncio.run(test_connection())