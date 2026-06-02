# Robotarium Hub 🤖🛰️

This repository contains the central core of **Robotarium**, a modular ecosystem designed to manage, telemetry, and control a fleet of up to 10 low-cost autonomous micro-robots.

The `RobotariumHub` acts as the central orchestrator or broker, handling all agent subscriptions and message forwarding. It unifies incoming data streams from different agents—whether they arrive directly via TCP/IP or through MQTT—and redistributes them to local clients or external dashboards.

## 🏗️ System Architecture

The hub utilizes a hybrid communication architecture to balance high performance with flexibility:

* **Control Commands (ZeroMQ REP/REQ):** The Hub exposes a synchronous port where agents can register themselves when powering up or joining the network.
* **Agent Telemetry (ZeroMQ SUB/PUB):** Each robot (via its Raspberry Pi) publishes its internal data (sensors, encoders, states) locally. The Hub dynamically subscribes to these publishers upon registration.
* **Data Redistribution (ZeroMQ PUB & MQTT):** The Hub unifies all incoming agent streams and rebroadcasts them under structured topics. This allows any mission client, GUI, or logging system to consume the fleet's data in real time.

[ Agent / Raspberry Pi ]  ---(ZMQ PUB: Data)----> [  Robotarium Hub  ]

[ Agent / Raspberry Pi ]  <--(ZMQ REQ: Hello)---- [ (Central Orchestration) ]

|

+---> (ZMQ PUB: Local Redistribution)

+---> (MQTT: Global Telemetry)

---

## 🛠️ Requirements & Installation

The hub is designed to run on Python 3. Make sure to install the required dependencies:

```bash
pip install pyzmq paho-mqtt
```

If you are using the MQTT bridge gateway, ensure you have an MQTT broker (such as Mosquitto or EMQX) running on your local machine (127.0.0.1:1883).

## 🚀 Hub Configuration & Operation

By default, the Hub binds to and manages the network interfaces, routing communications between the high-level coordinating server, the Raspberry Pi onboard agents, and the local low-level microcontrollers (such as Arduino MKR or Nano IoT). 

The Raspberry Pi acts as the main onboard processing unit: it computes complex algorithms and mission logic, receives telemetry from the Arduino over a serial connection, and communicates with the central Hub via TCP/IP.

| Interface / Protocol | Address / Port | Hub Role | Description |
| :--- | :--- | :--- | :--- |
| **ZeroMQ (REP)** | `tcp://192.168.10.1:5555` | `BIND` | Listens for registration requests (`hello`) from the Raspberry Pi onboard units. |
| **ZeroMQ (PUB)** | `tcp://192.168.10.1:5556` | `BIND` | Broadcasts aggregated telemetry received from the entire fleet. |
| **MQTT** | `127.0.0.1:1883` | `CLIENT` | Connects to the local broker for external data bridging and global telemetry tracking. |

### Agent Registration & Communication Flow

For a robot to successfully join the active network, the communication loop must be established hierarchically:

1. **Low-Level Control:** The Arduino handles the hardware layers—reading wheel encoders, interpreting the IMU, and regulating the H-bridge motor speeds via local serial packets (`Serial1`).
2. **Onboard Routing:** The Raspberry Pi reads this raw telemetry data, processes high-level mission directives, and coordinates with the central orchestrator.
3. **Hub Registration:** The Raspberry Pi registers itself by sending a JSON message to the Hub's command port (`5555`) with the following structure:

```json
{
  "source_id": "robot_05",
  "operation": "hello",
  "payload": {
    "url": "tcp://192.168.10.105:4444"
  }
}
```

## 📂 Generated Topic Structure

When the Hub processes a message of type `"data"`, it unpacks the internal payload and generates dynamic sub-topics to facilitate easy filtering for the clients:

`data / [metric_key] / [original_agent_topic]`

---

## 👥 Core Client Architecture (`/Clients`)

The `Clients` directory contains specialized mission scripts. To maintain modularity and avoid code duplication, all clients depend on two core shared modules: `agent.py` (which manages network synchronization with the Hub) and `logger_config.py` (which standardizes system-wide debugging).

### 1. Base Agent Module (`agent.py`)

The `Agent` class abstracts all the low-level ZeroMQ boilerplate networking. Every custom mission client inherits from this class to instantly gain connectivity with the `RobotariumHub`.

#### Key Features:
* **Automated Handshake:** On initialization, it automatically sends a `hello` command to the Hub's `REP` port (`5555`) to register its own local publication URL.
* **Dual-Socket Topology:** * A `REQ` (Request) socket to talk directly to the Hub's command center.
  * A `PUB` (Publish) socket to broadcast its own localized telemetry and sensor data streams.
* **Thread-Safe Data Transmission:** Includes built-in methods to send JSON-formatted payloads smoothly without blocking the main telemetry loop.

#### Registration JSON Payload Structure:
When an instance of `Agent` spins up, it automatically sends the following payload to the Hub:
```json
{
  "source_id": "<self.id>",
  "operation": "hello",
  "payload": {
    "url": "tcp://<local_ip>:<port>"
  }
}
```
### 2. Logging Configuration Module (`logger_config.py`)

To keep terminal outputs clean, readable, and unified across multiple robots running simultaneously, the `logger_config` module sets up a standardized logging structure using Python's native `logging` library.

#### Key Features:
* **Visual Log Levels:** Differentiates system states using standard markers (`INFO`, `DEBUG`, `WARNING`, `ERROR`).
* **Timestamped Tracking:** Every event is logged with millisecond precision, the name of the module that triggered it, and the specific log level.
* **Format Structure:**
  ```text
  %(asctime)s - %(name)s - %(levelname)s - %(message)s
  ```


### 🛠️ How to Implement a New Client

When creating a new mission script inside the `Clients` folder, use the following structural blueprint to extend the base architecture:

```python
from agent import Agent
from logger_config import setup_logger

# Initialize standard logging for this specific client mission
logger = setup_logger("MissionNameClient")

class CustomMissionClient(Agent):
    def __init__(self, agent_id, hub_ip, local_ip, port):
        # Pass networking parameters straight to the Base Agent
        super().__init__(agent_id, hub_ip, local_ip, port)
        logger.info(f"Client {agent_id} initialized successfully.")

    def broadcast_telemetry(self, metric_name, data):
        """Custom method to package and stream mission telemetry through the Hub"""
        topic = "data"
        payload = {
            "source_id": self.id,
            "topic": metric_name,
            "payload": data
        }
        # Send using the inherited ZMQ infrastructure
        self.socket.send_string(topic, flags=zmq.SNDMORE)
        self.socket.send_json(payload)
        logger.debug(f"Sent {metric_name} updates to Hub.")

```
---

## 🌐 Network Registry & Port Allocation

To avoid data collisions and address conflicts across the fleet (up to 10 robots), the Robotarium uses a strict port allocation strategy. High-level agents communicate locally via specific **ZeroMQ (ZMQ) PUB** ports, while remote or cloud-based orchestration leverages specialized **MQTT topics** mapped to the centralized broker.

### Master Port & Topic Assignment Table

| Subsystem / Agent | Network Protocol | Default Port | Primary Network Topic / Channel | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Robotarium Hub** | ZMQ (REP)<br>ZMQ (PUB) | `5555`<br>5556` | *Incoming Handshakes*<br> Aggregated Telemetry | Central orchestrator and data router. Baseline bind IP: `192.168.10.1`. |
| **Global MQTT Broker** | MQTT | `1883` | `#` (Wildcard root subscriber) | Standard local broker gateway (e.g., Mosquitto / EMQX). |
| **Vision Agent** | ZMQ (PUB) | `5559` | `vision/stitched` | Streams composite dual-overhead base64 MJPEG frames. |
| **ArUco Tracker Agent** | ZMQ (PUB) | *Dynamic* | `position` | Extracts and publishes spatial $(x, y, \theta)$ tracking packets. |
| **Arena Mapping Agent** | ZMQ (PUB) | *Dynamic* | `arena/state` | Translates pixels into metric (cm) layouts and monitors boundary polygons. |
| **Remote Control Agent** | ZMQ (PUB) | `5572` (Default) | `op_move_robot` | Manual teleoperation driver. Port parameter is configurable via CLI flags. |
| **Autonomous Bounce Agent** | ZMQ (PUB) | *Dynamic* | `agent/[ID]/move`<br>`agent/[ID]/turn` | Asynchronous FSM collision avoidance channel linked via MQTT bridge. |
| **Robot Fleet (01-10)** | Serial / WiFi | *Dynamic* | `agent/[robot_id]/...` | Individual robot units forward internal physical micro-controller payloads. |

> 💡 **Developer Rule:** When spinning up multiple instances of a `RemoteControl` or custom deployment script concurrently, verify that the active port execution flag (`--port XXXX`) maps to an unassigned sequence above `5560` to guarantee non-blocking ZeroMQ frame pipelines.
> 
---

## 👁️ Overhead Vision System & Calibration (`/Clients`)

The Robotarium features an overhead coordinate tracking and visual arena monitoring system using two fixed overhead cameras. Because a single camera cannot cover the full workspace without lens distortion or blind spots, this module uses **Homography Matrix Stitching** to fuse two distinct camera perspectives into a single, seamless global coordinate system.

[ Camera A Perspective ]    [ Camera B Perspective ]

\                             /

\                           /

[ calibration_stitching.py: Selects 4 overlapping points ]

|

( homography_matrix.npy )

|

[ vision_agent.py: Real-time Fused Global Frame ]

|

( ZMQ: vision/stitched ) ----> [ Robotarium Hub ]


### 1. Interactive Calibration Utility (`calibration_stitching.py`)

Before running the real-time agent, the system must calculate how to warp and align the image from Camera B onto the perspective plane of Camera A.

#### How It Works:
1. **Physical Setup:** Place 4 distinct optical markers in the overlapping field of view of both cameras inside the Robot Arena.
2. **Point Selection:** Run the script to open dual camera windows (`Camara A` and `Camara B`). Use your mouse to click the same 4 markers on both screens following **the exact same sequence** (e.g., clockwise starting from top-left).
3. **Homography Generation:** The script uses OpenCV’s `cv2.findHomography()` to calculate a `3x3` perspective transformation matrix.
4. **Storage:** The final matrix is exported as `homography_matrix.npy` to the root directory for live runtime rendering.

---

### 2. Matrix Validation & Stitching Test (`calibration2.py`)

A non-interactive script used to quickly test, preview, and adjust the homography projection alignment without restarting the initialization handshake.

#### Key Features:
* **Canvas Boundary Offset Optimization:** Warping an image can shift pixels out of standard window bounds, causing black masking artifact cutoffs. This script captures the warped corner matrices (`cv2.perspectiveTransform`) to automatically calculate horizontal (`offset_x`) and vertical (`offset_y`) canvas buffer padding.
* **Unified Workspace Array:** Translates and blends the final frame arrays into an optimized preview window size (`MAX_WIDTH` x `MAX_HEIGHT`).

---

### 3. Live Streaming Overhead Vision Agent (`vision_agent.py`)

This is the active mission client that runs continuously. It captures video feeds at low latency, warps them into the global workspace, encodes the final feed, and broadcasts it across the network.

#### Key Architectural Details:
* **Protocol Fulfillment:** It wraps the vision logic inside a `VisionDevice` class that perfectly implements the custom structural `Device` protocol required by `agent.py`.
* **Hardware Configuration & Optimization:** * Fixed frames-per-second (`CAP_PROP_FPS = 10` cap to reduce CPU overhead).
  * Minimum frame buffer size (`CAP_PROP_BUFFERSIZE = 1`) to eliminate streaming lag.
  * Explicit manual exposure configurations (`CAP_PROP_AUTO_EXPOSURE = 0.25`, `EXPOSURE = -7`) to suppress glare from reflective floors or external arena lightning changes.
* **Base64 String Network Serialization:** Raw numpy arrays cannot travel straight across JSON data payloads. The agent compresses the stitched output into a high-density JPEG buffer, encodes it into a standard text string via `base64.b64encode()`, and streams it on a dedicated topic string.

#### Outbound Message Metadata Schema:
* **Topic Channel:** `vision/stitched`
* **JSON Payload Payload:**
```json
{
  "source_id": "VisionSystem05",
  "topic": "vision/stitched",
  "payload": {
    "image": "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAIBAQEBAQIBAQECAgICAgQDAgICAgUEBAMEBgUGBgYF...",
    "width": 1920,
    "height": 1080
  }
}
```

🚀 Launching the Vision System

Step 1: Run Camera Alignment Calibration

Run the calibration script and select your 4 reference coordinates in sequence:
```Bash

python3 calibration_stitching.py
```

Step 2: Launch Live Network Stream Agent

To spin up the vision client onto the fleet network:
```Bash

python3 vision_agent.py
```

If you are deploying this headless (e.g., via SSH onto an arena-dedicated server without an active desktop display), pass the UI avoidance modifier tag:
```Bash

python3 vision_agent.py --no-gui
```

---

## 📍 ArUco Tracker Position System (`/Position`)

The `pos_agent.py` script acts as an active tracking client on the network. Instead of capturing video from a physical camera directly, it subscribes to the `vision/stitched` data topic broadcasted by the **Vision Agent**. It processes the base64-encoded composite frame string in real-time, extracts global spatial coordinate telemetry using **ArUco Markers**, and publishes the exact position/rotation data of each active robot back to the Hub.

[ Hub: vision/stitched ] ---> (pos_agent.py) ---> Decodes Base64 Image

|

Detects DICT_4X4_50 Markers

|

Computes (X, Y) & Theta (𝜽)

|

[ Hub: position ] <------------ Sends JSON Packet <-----+


### 1. Spatial Telemetry Processing Client (`pos_agent.py`)

This agent handles heavy computer vision detection pipelines asynchronously to keep network latency minimal.

#### Key Architectural Features:
* **Decoupled Queue Processing:** Incoming base64 frames are immediately captured from the network and pushed to an isolated thread-safe buffer (`queue.Queue()`). A background worker thread handles frame decoding and ArUco extraction to prevent the network communication thread from dropping frames due to processing spikes.
* **Optimized ArUco Parameters:** * Uses the standardized **`DICT_4X4_50`** dictionary (highly reliable for small micro-robots).
  * Tuning variables (`adaptiveThreshWinSizeMin`, `adaptiveThreshWinSizeMax`, and `minMarkerPerimeterRate`) are custom configured to accurately detect markers even if they are physically small or under fluctuating lighting profiles.
* **Orientation Tracking (𝜽):** Calculates the exact heading of the robot in radians by assessing the geometric vector between the marker's top corners (`corner[0]` and `corner[1]`).

#### Outbound Position Payload Schema:
When a valid marker is found in the workspace arena, the tracking information is routed back to the Hub under the target topic `position`:

* **Topic Channel:** `position`
* **JSON Payload:**

```json
{
  "source_id": "ArucoTracker",
  "topic": "position",
  "payload": {
    "robot_05": {
      "x": 420,
      "y": 680,
      "theta": 1.5708
    }
  }
}
```

Note: If multiple robots are visible inside the global stitched frame array simultaneously, they will all be mapped as distinct key entries (robot_01, robot_02, etc.) within the same payload packet.

### 🚀 Launching the Tracker Agent

Ensure that the main RobotariumHub and the overhead vision_agent.py are already running on the local network.

To start extracting global coordinates and broadcasting them:
```Bash

python3 pos_agent.py
```
To execute the tracker headlessly on a server without launching a local OpenCV debug monitoring window:

```Bash

python3 pos_agent.py --no-gui
```
---

## 🗺️ Arena Mapping & Metric Coordinate System (`/RobotArenaAgent`)

The `arena_agent.py` script acts as the workspace coordinator. Similar to the position tracker, it subscribes to the `vision/stitched` topic from the **Vision Agent**. However, its primary responsibility is two-fold: converting raw camera pixels into real-world metric dimensions (centimeters) and dynamically tracking the physical polygon boundaries of the Robot Arena setup.

[ Hub: vision/stitched ] ---> (arena_agent.py) ---> Decodes Base64 Image
|
Applies Lens Corner Matrix
|
Converts Pixels ➔ Metric (cm)
|
[ Hub: arena/state ] <--------- Sends JSON Packet <-----+


### 1. Workspace Metric Calibration (`arena_agent.py`)

This agent maps the workspace pixels to absolute physical dimensions using predefined physical constraints of your testing space:

#### Real-World Scaler Variables:
* **Arena Dimensions:** Width is bound to `419 cm` and Height to `140 cm`.
* **Pixel Transformation Shifts:** Employs manual spatial correction factors (`OFFSET_X = 854.14`, `OFFSET_Y = 434.92`) to isolate the active floor tracking canvas from peripheral image noise.
* **Proportional Multipliers (`SCALE_X`, `SCALE_Y`):** Standardizes real-time coordinate streams so any external pathfinding or collision avoidance application calculates steps based on centimeters rather than varying video frame matrices.

#### Key Spatial Features:
* **ArUco Boundary Mapping:** Uses specific corner tags to extract the exact corner geometry of the workspace perimeter.
* **Proximity Point Filtering:** Includes an automatic proximity distance-reduction algorithm (`filter_close_points`) utilizing a Euclidean distance check ($\sqrt{\Delta x^2 + \Delta y^2}$). If boundary corner markers are closer than a configured `min_dist`, they are merged to ensure clean, error-free outer boundary polygons for external path-planning clients.

#### Outbound Arena State Schema:
Processed geometric states are routed back to the Hub under the topic `arena/state`:

* **Topic Channel:** `arena/state`
* **JSON Payload:**
```json
{
  "source_id": "RobotArena",
  "topic": "arena/state",
  "payload": {
    "dimensions": {
      "width_cm": 419.0,
      "height_cm": 140.0
    },
    "corners": [
      [0.0, 0.0],
      [419.0, 0.0],
      [419.0, 140.0],
      [0.0, 140.0]
    ],
    "area_m2": 5.866
  }
}
```

### 🚀 Launching the Arena Mapping Agent

Make sure that the main RobotariumHub and the vision_agent.py are active on your subnet.

To launch the arena coordinates translator:
```Bash

python3 arena_agent.py
```

To execute headlessly on an arena terminal server without firing up an active debugging UI display window:
```Bash

python3 arena_agent.py --no-gui
```
---

## 🎮 Remote Control & Teleoperation System (`/RemoteControl`)

The `remote_control_agent.py` script is a dynamic teleoperation client that lets an operator pilot any designated micro-robot in the fleet. It captures immediate keystroke events from the terminal using low-level I/O (`termios` and `tty`), maps them to linear ($v$) and angular ($w$) velocity targets, logs the command history to a local CSV file, and broadcasts the motion frames over the network.

[ Terminal Keystrokes ]  ──► (GetKey) ──► Maps to (v, w)

│

┌──────────────────────────────────────┴──────────────────────────────────────┐

▼                                                                             ▼

(Local CSV Logging)                                                        (ZMQ command packet)

[ robot_[ID]_log.csv ]                                                              │

▼

[ Hub: op_move_robot ]


### 1. Non-Blocking Key Capture (`GetKey`)

To make steering highly responsive, the script implements a custom `GetKey` class. 
* By using `termios.tcgetattr` and `tty.setraw`, it puts the terminal into **raw mode**.
* This forces the system to register keystrokes instantly as they are pressed, removing the default requirement where a user must hit `Enter` to submit an input string.
* It uses `select.select` with a strict `0.1s` timeout window to ensure the keyboard listening process never blocks the rest of the execution loop.

### 2. Teleoperation Controller & Mapping (`Teleoperator`)

The control scheme uses a standard layout to manipulate target velocities incrementally:

| Key Press | Intended Motion | Resulting Command Action |
| :--- | :--- | :--- |
| **`w`** | Move Forward | Increments Linear Velocity ($v$) |
| **`s`** | Move Backward | Decrements Linear Velocity ($v$) |
| **`a`** | Spin Left | Increments Angular Velocity ($w$) |
| **`d`** | Spin Right | Decrements Angular Velocity ($w$) |
| **`space`** | Emergency Brake | Hard reset: zeros out both $v$ and $w$ immediately |

#### Automated Data Logging:
Every single time a motion command modifies the system speed vectors, the `Teleoperator` automatically appends a detailed tracking record into a local file (`/logs/robot_[ID]_log_[TIMESTAMP].csv`). The CSV schema stores:
`Timestamp, Target Robot ID, Linear Velocity (v), Angular Velocity (w)`

#### Outbound Control Payload Schema:
Commands are packaged as operational JSON blocks and routed to the Hub:

* **Topic Channel:** Based on standard operation rules (`op_move_robot`)
* **JSON Payload:**
```json
{
  "source_id": "TeleopAgent_06",
  "topic": "control",
  "payload": {
    "robot_id": 6,
    "v": 0.4,
    "w": -0.2
  }
}
```
### 🚀 Launching the Teleoperation Agent

The teleoperation script uses command-line arguments to allow piloting different units without modifying code strings.

To run the controller for Robot ID 6 on its designated network port:
```Bash

python3 remote_control_agent.py --robot_id 6 --port 5572
```

Available CLI Arguments:

    -r, --robot_id: The numeric target identifier of the robot you want to control (e.g., 5, 6, 8). Default is 6.

    -p, --port: The specific ZeroMQ data port allocated to this agent's network channel. Default is 5572.

---

## 🎛️ Autonomous Wall-Bouncing System (`/BounceRobot`)

The `agent_bounce.py` script implements a high-level autonomous navigation client utilizing an asynchronous **Finite State Machine (FSM)**. It bridges an inbound cloud/global MQTT channel over to the local high-speed ZeroMQ (`ZMQ`) channels, processes real-time telemetry inputs, tracks arena proximity relative to the bounding walls, and computes autonomous evasive actions (bouncing) to safeguard the fleet hardware.

[ Inbound MQTT Telemetry / Control ]

│

▼

(on_message callback)

│

(Thread-Safe Command Queue)

│

▼

[ FSM Navigation State ] ───► Calculates Wall Distances

│

┌─────────────┴─────────────┐

▼                           ▼

(State: ADVANCE)          (State: BOUNCE / TURN)

[ v=0.3, w=0.0 ]          [ Precise Angular Rotation ]

│                           │

└─────────────┬─────────────┘

│

▼

[ ZMQ: Outbound Packet ] ───► [ Hub: agent/[ID]/move ]


### 1. Asynchronous Thread-Safe Queue Dispatcher

Because MQTT callbacks execute on an independent background network thread, handling raw navigation controls immediately inside the callback can stall execution loops. 
* This agent features a thread-safe implementation using Python’s native **`queue.Queue()`**.
* The local MQTT listener handles network packets and pipes raw payloads into the tracking queue.
* The main system thread continuously polls the queue, resolves state evaluations, and dispatches compiled control packets to the Hub over ZeroMQ without synchronization deadlocks.

### 2. Bounding Arena Collision Avoidance FSM

The engine tracks absolute position coordinates ($\mathbf{X, Y}$) streamed via telemetries and runs structural boundary constraints matching the arena size profile (`419cm x 140cm`). 

#### Navigation State Profiles:
* **`ADVANCE` State:** The robot moves forward cleanly at a safe velocity baseline ($v = 0.3\,\text{m/s}$, $w = 0.0$). 
* **`BOUNCE` (Evasive Rotation) State:** Triggered immediately when proximity distance to any outer perimeter boundary drops below a critical safety threshold. The agent calculates the closest wall vector and executes a precise angular target spin (`agent/[ID]/turn`) in the exact opposite direction before reverting back to the forward motion profile.
* **`CRITICAL_STOP` State:** An emergency override fallback that cuts driver power completely if telemetry positioning anomalies drop below a physical structural limit, preventing catastrophic wall impacts.

#### Outbound Network Bridging Schemas:
The dispatcher dynamically resolves target topics based on operational requirements:

* **Velocity Movement Topic:** `agent/[robot_id]/move`
* **JSON Velocity Payload:**
```json
{
  "v": 0.3,
  "w": 0.0
}
```
* **Precise Rotation Topic:** `agent/[robot_id]/turn`
* **JSON Rotation Payload:**

```json

{
  "ang": 95.0
}
```

Note: The ang parameter instructs the onboard microcontroller to perform a precise orientation correction tracking sequence measured in degrees.

### 🚀 Launching the Bouncing Agent

Ensure your master RobotariumHub, tracking elements, and global MQTT broker configurations are accessible on the server.

To run the autonomous bouncing logic script for a designated unit (e.g., Robot ID 5):
```Bash

python3 agent_bounce.py
```




