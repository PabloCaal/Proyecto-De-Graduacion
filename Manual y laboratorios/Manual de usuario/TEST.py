import logging
import time
import sys
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander

cflib.crtp.init_drivers(enable_debug_driver=False)
logging.basicConfig(level=logging.CRITICAL)
   
def connect(uri):
    try:        
        scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
        scf.open_link()
        sys.stdout.flush()
        return scf
    except Exception as e:
        if 'Cannot find a Crazyradio Dongle' in str(e):
            print(f"Error: Crazyradio Dongle not found. Ensure the dongle is connected properly.")
        elif 'Connection refused' in str(e):
            print(f"Error: Connection to Crazyflie was refused. Check if the Crazyflie is powered on and in range.")
        else:
            print(f"General error occurred while trying to connect to Crazyflie. Error details: {str(e)}")

def disconnect(scf):
    try:
        if scf:
            scf.close_link()
        else:
            print(f"Error: Invalid SyncCrazyflie object. No connection to close.")

    except Exception as e:
        print(f"Error: An issue occurred while disconnecting from Crazyflie. Error details: {str(e)}")

def detect_flow_deck(scf):
    try:
        flow_deck_detected = scf.cf.param.get_value('deck.bcFlow2')

        if flow_deck_detected == '1':
            return 1
        else:
            return 0
    except Exception as e:
        print(f"Error: An issue occurred while detecting the Flow Deck. Error details: {str(e)}")

def main():
    # Conectar al dron
    try:
        scf = connect("radio://0/80/2M/E7E7E7E7E7")
        print("Connection to Crazyflie established successfully.")

        time.sleep(1.0)

        # Verificar si el Flow Deck está conectado
        if detect_flow_deck(scf):
            print("Flow Deck detected successfully.")
        else:
            print("Flow Deck not detected. Please verify that it is installed properly.")

        time.sleep(1.0)

        # Desconectar del Crazyflie
        disconnect(scf)
        print("Successfully disconnected from Crazyflie.")        

    except Exception as e:
        print(f"Error al intentar establecer la conexión con el dron Crazyflie: {e}")

if __name__ == '__main__':
    main()
