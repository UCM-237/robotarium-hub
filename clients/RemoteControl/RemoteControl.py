import struct
import time
import sys
import tty
import termios
import select
import agent
import  xml.etree.ElementTree as ET

# Importamos Robot que a su vez hereda de Agent
from remote_control_agent import Robot

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

class RemoteControl(Robot):
    def __init__(self):
        # Al llamar a super().__init__, Agent leerá el XML y abrirá el Serial automáticamente
        # según AgentConfiguration.xml
        self.agentParameters={}
        self.AgentName="Arwen"
        self.agentParameters['AgentId']="3"
        self.port="USB2.0-Serial"
        self.AgentIP="192.148.10.49"
        self.device="Robot"

        agent.Agent.__init__(self,self.device,self.AgentName,self.AgentIP)
        self.gk = GetKey()

    def run_teleop(self):
        v_lin, v_ang = 0.0, 0.0
        
        print(f"\n--- CONTROL REMOTO ROBOT ID: {self.agentParameters['AgentId']} ---")
        print("W/S/A/D: Mover | Espacio: STOP | Q: Salir")
        print("----------------------------------------------\n")

        try:
            while True:
                key = self.gk.get_key()
                
                if key == 'w': v_lin += 0.5
                elif key == 's': v_lin -= 0.5
                elif key == 'a': v_ang -= 0.3
                elif key == 'd': v_ang += 0.3
                elif key == ' ':
                    v_lin, v_ang = 0.0, 0.0
                    # Usamos los OP codes definidos en la clase padre Agent
                    self.ArduinoSerialWrite(self.OP_STOP_ROBOT, 0, b'')
                elif key == 'q':
                    break

                if key in ['w', 's', 'a', 'd', ' ']:
                    # Limitamos velocidades
                    v_lin = max(min(v_lin, 13.5), -13.5)
                    # Empaquetamos según el protocolo esperado por el Arduino
                    data_move = struct.pack('<ff', float(v_lin), float(v_ang))
                    self.ArduinoSerialWrite(self.OP_MOVE_ROBOT, len(data_move), data_move)
                    print(f"\rCMD: V:{v_lin:5.2f} | W:{v_ang:5.2f} ", end='', flush=True)

                # El método update() de Robot o Agent puede manejar la recepción
                # Si quieres ver qué dice el Arduino en tiempo real:
                if self.arduino and self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if line: print(f"\n[ARDUINO] {line}")
                
                time.sleep(0.01)

        except Exception as e:
            print(f"\n[!] Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.gk.settings)

if __name__ == "__main__":
    # Asegúrate de que AgentConfiguration.xml esté en la misma carpeta
    remote = RemoteControl()
    # Verificamos si el puerto serie se abrió correctamente en el constructor de Agent
    if remote.arduino and remote.arduino.is_open:
        remote.run_teleop()
    else:
        print("[!] No se pudo abrir el puerto serie definido en el XML.")
