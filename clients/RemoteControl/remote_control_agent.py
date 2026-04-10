# -*- coding: UTF-8 -*-
import sys
import tty
import termios
import select
import time
from pi_agent_limits import Robot  # Importamos la clase que ya funciona
from agent import Agent

class GetKey:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

# Creamos una clase nueva que EXTIEUNDE a la que ya funciona
class TeleopLimits(Robot):
    def __init__(self, agent):
        super().__init__(agent) # Ejecuta el init original (abre serial, etc.)
        self.gk = GetKey()
        self.v_lin = 0.0
        self.v_ang = 0.0
        self.ang=0

    def run_teleop_loop(self):
        print("\n" + "="*40)
        print("   TELEOP ROBOTARIUM (PI_AGENT_LIMITS)")
        print("="*40)
        print(" W/S: Lineal | A/D: Angular | G/H: Angulo giro | Espacio: STOP")
        print(" Q: Salir")
        
        try:
            while True:
                key = self.gk.get_key()
                
                if key == 'w': self.v_lin += 0.5
                elif key == 's': self.v_lin -= 0.5
                elif key == 'a': self.v_ang -= 0.1
                elif key == 'd': self.v_ang += 0.1
                elif key == 'g': self.ang +=5
                elif key == 'h': self.ang -=5
                elif key == ' ':
                    self.v_lin, self.v_ang = 0.0, 0.0
                    self.ArduinoSerialWrite(self.OP_STOP_ROBOT, 0, b'')
                elif key == 'q':
                    break

                if key in ['w', 's', 'a','d',' ']:
                    # Usamos el método move_robot que ya está definido en pi_agent_limits.py
                    # Ese método ya hace el empaquetado y envío al Arduino
                    print(f"\rV: {self.v_lin:5.2f} | W: {self.v_ang:5.2f} ", end='', flush=True)
                    vl=self.v_lin-(13.1/2.0)*self.v_ang
                    vr=2*self.v_lin-vl                    
                    wl=vl/3.35
                    wr=vr/3.35
                    print(f"\r wr={wr}, wl={wl} (rad/s)",end='',flush=True)
                    self.move_robot(wl,wr)
                elif key in ['g','h']:
                    print(f"Ang. giro: {self.ang} (grad)",end='',flush=True)
                    self.turn_robot(self.ang)    

                time.sleep(0.01)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.gk.settings)

if __name__ == "__main__":
    # 1. Configuración idéntica a pi_agent_limits.py
    robot_class = Robot
    robot_class.parseConfigurations(robot_class)
    
    # 2. Arrancamos el Agente usando nuestra NUEVA clase TeleopLimits
    agent_instance = Agent(
        device_class=TeleopLimits, 
        id=robot_class.agentParameters['AgentName'],
        ip=robot_class.communicationParameters['AgentIp'],
        cmd_port=robot_class.communicationParameters['AgentCmdPort'],
        data_port=robot_class.communicationParameters['AgentDataPort'],
        hub_ip=robot_class.communicationParameters['HubIp'],
        hub_cmd_port=robot_class.communicationParameters['HubCmdPort'],
        hub_data_port=robot_class.communicationParameters['HubDataPort']
    )

    # 3. Ejecutamos el bucle de teleoperación
    agent_instance.device.run_teleop_loop()
