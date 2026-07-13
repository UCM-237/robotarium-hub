# test_network_delay.py
import paho.mqtt.client as mqtt
import json
import time

BROKER = "192.168.10.1"
PUERTO = 1883

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        
        # Si el mensaje contiene el eco del timestamp del servidor
        if 't_srv' in payload:
            t_salida = payload['t_srv']
            t_llegada = int(time.time() * 1000) # Tiempo actual en el servidor
            
            # El RTT (Round Trip Time) mide ida + procesamiento + vuelta
            rtt = t_llegada - t_salida
            # El retraso estimado de ida unidireccional es aproximadamente la mitad
            one_way_delay = rtt / 2
            
            print(f"[ROBOT {payload.get('id', 'U')}] RTT de red: {rtt} ms | Retraso estimado ida: {one_way_delay} ms")
            
            if one_way_delay > 50:
                print(f"  --> ALERT: ¡Retraso crítico detectado! ({one_way_delay} ms)")
    except Exception as e:
        pass

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, PUERTO, 60)

# Nos suscribimos a la telemetría de todos los robots para auditar la red
client.subscribe("+/pos")
client.subscribe("agent/+/move") 
client.loop_forever()