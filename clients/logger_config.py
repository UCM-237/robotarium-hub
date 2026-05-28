import logging
import os
import colorlog

def setup_logger(agent_name,console_level=logging.INFO):
    # 1. Definir los formatos de texto
    # Formato para la consola (con la etiqueta de colorlog)
    console_format = "%(log_color)s%(asctime)s - [%(name)s] - %(levelname)s - %(message)s%(reset)s"
    # Formato para el archivo (limpio, sin códigos de color)
    file_format = "%(asctime)s.%(msecs)03d - [%(name)s] - %(levelname)s - %(message)s"

    date_format = '%Y-%m-%d %H:%M:%S'

    # 2. Obtener el logger del agente
    logger = logging.getLogger(agent_name)
    logger.setLevel(logging.DEBUG)

    # Evitar duplicar manejadores si se vuelve a llamar a la función
    if not logger.handlers:
        
        # --- MANEJADOR 1: CONSOLA (CON COLORES) ---
        LOG_COLORS = {
            'DEBUG':    'cyan',
            'INFO':     'green',
            'WARNING':  'yellow',
            'ERROR':    'red',
            'CRITICAL': 'bold_red',
        }
        
        console_formatter = colorlog.ColoredFormatter(
            console_format,
            datefmt=date_format,
            log_colors=LOG_COLORS
        )
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(console_formatter)

        # ---> AQUÍ CONFIGURAMOS EL FILTRO DE LA CONSOLA <---
        # Solo mostrará los mensajes de este nivel o superior (ej. WARNING, ERROR)
        console_handler.setLevel(console_level)
        
        logger.addHandler(console_handler)
        # Creamos una carpeta para los logs si no existe 
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        # El archivo se llamará, por ejemplo: logs/Robot_05.log
        file_path = os.path.join(log_dir, f"{agent_name}.log")
        
        file_formatter = logging.Formatter(file_format, datefmt=date_format)
        file_handler = logging.FileHandler(file_path, encoding='utf-8')
        file_handler.setFormatter(file_formatter)
        # ---> AQUÍ CONFIGURAMOS EL FILTRO DEL ARCHIVO <---
        # Forzamos a que el archivo SIEMPRE guarde todo desde DEBUG
        file_handler.setLevel(logging.DEBUG)
        logger.addHandler(file_handler)

    return logger