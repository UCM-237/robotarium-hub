# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import zmq
import json
import random

from paho.mqtt import client as mqtt_client
from time import sleep
from threading import Thread

# Configure logs
logging.basicConfig(level=logging.DEBUG)

CMD_PORT = 5555
DATA_PORT = 5556

# MQTT Config
MQTT_BROKER = '127.0.0.1'
MQTT_PORT = 1883
MQTT_TOPIC = "#"
# Generate a Client ID with the publish prefix.
MQTT_CLIENT_ID = f'publish-{random.randint(0, 1000)}'
# MQTT_USERNAME = 'emqx'
# MQTT_PASSWORD = 'public'


class RobotariumHub:
    def __init__(self):
        self.OPS = {
        'hello': self.add_agent,
        }
        self.context = zmq.Context()
        self.commands_socket = self.context.socket(zmq.REP)
        self.commands_socket.bind(f'tcp://192.168.10.1:{CMD_PORT}')
        self.data_socket = self.context.socket(zmq.PUB)
        self.data_socket.bind(f'tcp://192.168.10.1:{DATA_PORT}')
        self.agents = {}
        self.camera = {}
        self.running = True
        self.accepting = Thread(target=self.accept, args=())
        self.accepting.start()
        self.poller = zmq.Poller()
        self.listening = Thread(target=self.listen, args=())
        self.listening.start()
        #self.mqtt_client = self.connect_mqtt()

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

            client = mqtt_client.Client(MQTT_CLIENT_ID)
            # client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
            client.on_connect = on_connect
            client.connect(MQTT_BROKER, MQTT_PORT)
            client.loop_start()
            client.subscribe('#')
            client.on_message = self.on_mqtt_message
            return client
        

    def on_mqtt_message(self, client, userdate, msg):
        self.data_socket.send_string(msg.topic, flags=zmq.SNDMORE)
        self.data_socket.send_string(msg.payload.decode())

    def add_agent(self, id, url):
        #logging.info(f'Agent {id} registered with url {url}')
        self.agents[id] = {
            'url': url,
            'socket': self.context.socket(zmq.SUB)
        }
        self.agents[id]['socket'].connect(url)
        self.agents[id]['socket'].subscribe('')
        self.poller.register(self.agents[id]['socket'], zmq.POLLIN)

    def listen(self):
        logging.info(f'Listening to agents')
        while self.running:
            socks = dict(self.poller.poll(100))
            for a in self.agents:
                s = self.agents[a]['socket']
                if s in socks and socks[s] == zmq.POLLIN:
                    try:
                        topic = s.recv_string()
                        message = s.recv_json()
                    except json.JSONDecodeError:
                        logging.error(f'Agent {id} sent invalid JSON')

                    if topic == "data":
                        data = message["payload"]
                        print(data)
                        for k in data:
                            new_topic = f'{topic}/{k}/{message["topic"]}'
                            self.data_socket.send_string(new_topic, flags=zmq.SNDMORE)
                            self.data_socket.send_json(data[k])
                            
                    else:
                        self.data_socket.send_string(topic, flags=zmq.SNDMORE)
                        self.data_socket.send_json(message)
                        print(topic)
                        print(message)
                    
                    #self.mqtt_client.publish(topic, json.dumps(message))
                    

    def accept(self):
        '''Wait for next request from client'''
        while self.running:
            sleep(0.1)
            logging.info("Robotarium is waiting for new agents")
            message = self.commands_socket.recv_json()
            logging.info(f'Received request from {message["source_id"]}')
            self.OPS[message['operation']](message['source_id'], message['payload']['url'])
            self.commands_socket.send_json({ 'result': 'ok' })


def subscribe(client: mqtt_client):
  def on_message(client, userdata, msg):
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

  #client.subscribe(topic)
  client.on_message = on_message


if __name__ == "__main__":
  hub = RobotariumHub()
