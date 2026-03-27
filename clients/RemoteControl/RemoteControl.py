import struct
import time
from curtsies import Input
from remote_control_agent import Robot

class RemoteControl(Robot):
    def __init__(self):
        # Sincronizamos con tus constantes de remote_control_agent.py
        self.agentParameters = {'AgentId': 5} 
        self.INIT_FLAG = 112 
        self.arduino = None
        self.connect_serial()

    def connect_serial(self, port='/dev/ttyACM0', baud=115200):
        import serial
        try:
            self.arduino = serial.Serial(port, baud, timeout=0.1)
            print(f"[*] Conectado al Arduino en {port}")
        except Exception as e:
            print(f"[!] Error: {e}")

    def send_turn_angle(self, angle):
        """Envía el comando OP_TURN_ROBOT con un float (4 bytes)"""
        try:
            data = struct.pack('<f', float(angle))
            self.ArduinoSerialWrite(self.OP_TURN_ROBOT, len(data), data)
            print(f"\n[OK] Enviado giro de {angle} unidades.")
        except ValueError:
            print("\n[!] Error: Introduce un número válido.")

    def run_teleop(self):
        v_lin, v_ang = 0.0, 0.0
        
        print("\n--- DEPURACIÓN DE GIRO ROBOTARIUM ---")
        print("W/S/A/D: Control manual | Espacio: STOP")
        print("T: INTRODUCIR ÁNGULO ESPECÍFICO")
        print("Q: Salir")
        print("--------------------------------------\n")

        with Input(keynames='curses') as input_generator:
            for key in input_generator:
                if key == 'w': v_lin += 0.5
                elif key == 's': v_lin -= 0.5
                elif key == 'a': v_ang -= 0.3
                elif key == 'd': v_ang += 0.3
                elif key == ' ':
                    v_lin, v_ang = 0.0, 0.0
                    self.ArduinoSerialWrite(self.OP_STOP_ROBOT, 0, b'')
                
                # --- LÓGICA DE ENTRADA MANUAL ---
                elif key == 't':
                    # Limpiamos buffer de entrada y pedimos valor
                    print("\n" + "="*30)
                    angle_input = input("Introduce el ángulo de giro (float): ")
                    self.send_turn_angle(angle_input)
                    print("="*30 + "\n")
                    # Al terminar, volvemos al loop de curtsies automáticamente
                
                elif key == 'q':
                    break

                # Envío de comando de movimiento continuo
                v_lin = max(min(v_lin, 13.5), -13.5)
                data_move = struct.pack('<ff', float(v_lin), float(v_ang))
                self.ArduinoSerialWrite(self.OP_MOVE_ROBOT, len(data_move), data_move)
                
                print(f"\rCMD: Lin {v_lin:5.2f} | Ang {v_ang:5.2f} | 'T' para girar", end='', flush=True)

                # Leer telemetría para ver si el giro terminó o hubo error
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if line: print(f"\n[ARDUINO] {line}")

if __name__ == "__main__":
    remote = RemoteControl()
    if remote.arduino:
        remote.run_teleop()