# -*- coding: UTF-8 -*-
#!/bin/python3
import array
from serial import Serial, SerialException, serial_for_url
from serial.tools import list_ports
from threading import Thread
import logging

import time

from threading import Thread

from agent import Agent

# Configure logs
logging.basicConfig(level=logging.INFO)

# Who I am
AGENT_ID = 'Algorithm'
AGENT_IP = '192.168.1.128'
AGENT_CMD_PORT = 5563
AGENT_DATA_PORT = 5564
# Where the server is 
HUB_IP = '192.168.1.130'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556


class Algorithm:
  
  state: dict = {}
  
  def __init__(self, agent: Agent) -> None:
    self.agent = agent
    self.Position={}
  
  def connect(self):
    logging.debug('starting decice')
    
  
  def rendevouz(self,agent)-> None:
    print(agent)
  
  def on_data(self,topic: str, message: str) ->None:

    try:
      _, agent, topic = topic.split('/')
    except:
      print("invalid message")
      return
    # if agent not in self.state:
    #   self.state[agent]={}
    # self.state[agent][topic] = ast.literal_eval(message)
    if agent not in self.Position:
      self.thread = Thread(target=self.rendevouz(agent))
      self.thread.start()
 
    if topic == "position":
      self.Position[agent]=message
      


if __name__ == "__main__":
  agent = Agent(
    device_class=Algorithm,
    id=AGENT_ID,
    ip=AGENT_IP,
    cmd_port=AGENT_CMD_PORT,
    data_port=AGENT_DATA_PORT,
    hub_ip= HUB_IP,
    hub_cmd_port=HUB_CMD_PORT,
    hub_data_port=HUB_DATA_PORT,
  )
  
  
 