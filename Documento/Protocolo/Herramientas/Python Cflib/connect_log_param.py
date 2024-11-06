"""
UNIVERSIDAD DEL VALLE DE GUATEMALA
DISEÑO E INNOVACIÓN EN INGENIERÍA MECÁNICA
PROYECTO DE GRADUACIÓN

PABLO JAVIER CAAL LEIVA 20538
"""

# standard python libraries.
import logging
import time

# libraries for cflib
import cflib.crtp   # for scanning for Crazyflies instances
from cflib.crazyflie import Crazyflie   # used to easily connect/send/receive data from a Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
#  is a wrapper around the “normal” Crazyflie class. It handles the asynchronous nature of the Crazyflie API and turns it into blocking function.
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        simple_connect()


def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")