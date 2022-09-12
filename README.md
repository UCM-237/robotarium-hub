# robotarium-hub
A control hub for the Robotarium.

# Definitions
An agent is an element the robotarium should be aware of (a mobile robot).

## Purpose
The control hub is responsible of:
- Tracking the state robots in the Robotarium
- Gathering data from the agents
- Providing an API for clients to interact with the Robotarium

## How the control hub works
The control hub keeps a list of agents. To that end, every new agent that comes into the system must ask the hub to be registered. Once the agent is registered, the hub start to receive data from the agent and is able to send commands 

## Messages
All the messages that are exchanged between the agents and the hub are codified in JSON (JavaScript Object Notation) and have the following structure:
```
{
  'timestamp': int,
  'source_id': str | int,
  'operation': str,
  'payload': dict
}
```
For instance,
```
{
  'timestamp': 1663008404333,
  'source_id': 2,
  'operation': 'hello',
  'payload': { 'url':'tcp://127.0.0.1:5556' }
}
```

## Registering a new agent
By default, the control hub is listening to the port 5555. New agents must notify their existence sending a request message where the operation *id* is 'hello', and the *payload* is a dict that contains the data streaming *url*, the list of supported *actions*, and the list/description of the measurements reported by the agent.
An example of such message is,
```
{
  'timestamp': '',
  'source_id': ''.
  'operation': 'hello',
  'payload': {
    'url': 'tcp://x.x.x.x:5556', # The data streaming url
    'actions': ['move', ...],
    'measurements': {
      'x': {'description':'X axis position measured in centimeters.', 'min':0, 'max':300, 'precision':0.01},
      'y': {'description':'Y axis position measured in centimeters.', 'min':0, 'max':300, 'precision':0.01},
      ...
    }
  }
}
```


