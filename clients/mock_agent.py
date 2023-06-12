# -*- coding: UTF-8 -*-
#!/bin/python3
import logging
import time

from agent import Agent
from threading import Thread

# Configure logs
logging.basicConfig(level=logging.DEBUG)

# Who I am
AGENT_ID = 'Robot1'
AGENT_IP = '127.0.0.1'
AGENT_CMD_PORT = 5557
AGENT_DATA_PORT = 5558
# Where the server is 
SERVER_IP = '127.0.0.1'
SERVER_CMD_PORT = 5555
SERVER_DATA_PORT = 5556

class Robot:
  '''Encapsulates the communication with Arduino'''
  connected: bool = False
  thread: Thread = None
  listeners: list = None

  def __init__(self, listeners: list) -> None:
    self.parsers = {}
    self.operations = {}
    self.listeners = listeners


  def connect(self) -> None:
    logging.debug('Starting device')
    self.thread = Thread(target=self.update)
    self.thread.start()


  def update(self) -> None:
    '''Parse data received from the robot'''
    print('Starting Update thread')
    while True:
      try:
        time.sleep(2)
        measurement = '{"bat":3}'
        for l in self.listeners:
          l.send_measurement(measurement)
      except (ValueError, TypeError) as e:
        print('[WARNING] Ignoring invalid data from hardware')
        print(e)


  def parse(self, operation: int, data: bytes) -> dict:
    if operation not in self.parsers:
      raise ValueError(f'Undefined operation {operation}')
    return self.parsers[operation](data)


  def exec(self, operation: str, **kwargs) -> None:
    if operation not in self.operations:
      raise ValueError(f'Undefined operation {operation}')
    self.operations[operation]['method'](**kwargs)



if __name__ == "__main__":
    end = False
    # Wait for commands
    agent = Agent(
      id=AGENT_ID,
      device_class=Robot,
      ip=AGENT_IP,
      cmd_port=AGENT_CMD_PORT,
      data_port=AGENT_DATA_PORT,
      server_cmd_port=SERVER_CMD_PORT,
      server_data_port=SERVER_DATA_PORT,
    )
    agent.register()
    logging.info(f'Agent {agent.id} is listening')
    while not end:
      #  Wait for next request
      try:
        message = agent.accept_command()
        logging.info(f'Receiving command from server')
      except ValueError:
        logging.warning('Offending command')
