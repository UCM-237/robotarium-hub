# -*- coding: UTF-8 -*-
#!/bin/python3
import logging

from agent import Agent
from threading import Thread
from time import sleep

# Configure logs
logging.basicConfig(level=logging.DEBUG)

# Who I am
AGENT_ID = 'ChargeManager'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5559
AGENT_DATA_PORT = 5560
# Where the server is 
HUB_IP = '127.0.0.1'
HUB_CMD_PORT = 5555
HUB_DATA_PORT = 5556

class DummyChargingCoordinator:
  '''A dummy charging coordinator'''
  thread: Thread = None
  agent: Agent = None

  def __init__(self, agent: Agent) -> None:
    self.agent = agent


  def connect(self) -> None:
    logging.debug('Starting device')
    self.thread = Thread(target=self.update)
    self.thread.start()


  def update(self) -> None:
    '''Parse data received from the robot'''
    print('Starting Update thread')
    while True:
      sleep(10)
      self.agent.send('control/Robot_1/move', { 'v_left': 1, 'v_right': -1 })
      self.agent.send('control/Robot_2/move', { 'v_left': 1, 'v_right': -1 })


  def on_data(self, topic: str, message: str) -> None:
    print(f'Received data from {topic}: {message}')



if __name__ == "__main__":
  # Wait for commands
  agent = Agent(
    device_class=DummyChargingCoordinator,
    id=AGENT_ID,
    ip=AGENT_IP,
    cmd_port=AGENT_CMD_PORT,
    data_port=AGENT_DATA_PORT,
    hub_cmd_port=HUB_CMD_PORT,
    hub_data_port=HUB_DATA_PORT,
  )
  logging.info(f'Agent {agent.id} is listening')