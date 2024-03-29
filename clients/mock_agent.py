# -*- coding: UTF-8 -*-
#!/bin/python3
import logging

from agent import Agent
from threading import Thread
from time import sleep

# Configure logs
logging.basicConfig(level=logging.DEBUG)

# Who I am
AGENT_ID = 'Robot_1'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5557
AGENT_DATA_PORT = 5558
# Where the server is 
HUB_IP = '127.0.0.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556

class DummyDevice:
  '''A dummy device for testing the connections'''
  thread: Thread = None
  agent: Agent = None

  def __init__(self, agent: Agent) -> None:
    self.agent = agent


  def connect(self) -> None:
    logging.debug('Starting device')
    self.thread = Thread(target=self.update)
    self.thread.start()


  def update(self) -> None:
    '''Generate mocked measurements'''
    print('Starting Update thread')
    while True:
      sleep(5)
      measurement = { 'battery': 3 }
      self.agent.send_measurement(measurement)

  
  def on_data(self, topic: str, message: str) -> None:
    print(f'Received data from {topic}: {message}')



if __name__ == "__main__":
    # Wait for commands
    agent = Agent(
      id=AGENT_ID,
      device_class=DummyDevice,
      ip=AGENT_IP,
      cmd_port=AGENT_CMD_PORT,
      data_port=AGENT_DATA_PORT,
      hub_cmd_port=HUB_CMD_PORT,
      hub_data_port=HUB_DATA_PORT,
    )
    # agent.register()
    logging.info(f'Agent {agent.id} is listening')