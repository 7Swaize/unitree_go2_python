import time
import sys
import struct
import math
import threading

from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ # specific to using Go2
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ # specific to using Go2
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

from dataclasses import dataclass


class UnitreeRemoteController:
        def __init__(self):
            # key
            self.Lx = 0           
            self.Rx = 0            
            self.Ry = 0            
            self.Ly = 0

            # button
            self.L1 = 0
            self.L2 = 0
            self.R1 = 0
            self.R2 = 0
            self.A = 0
            self.B = 0
            self.X = 0
            self.Y = 0
            self.Up = 0
            self.Down = 0
            self.Left = 0
            self.Right = 0
            self.Select = 0
            self.F1 = 0
            self.F3 = 0
            self.Start = 0

        def parse_botton(self, data1, data2):
            # uint8_t bit seq, each flag representing an input.
            self.R1 = (data1 >> 0) & 1
            self.L1 = (data1 >> 1) & 1
            self.Start = (data1 >> 2) & 1
            self.Select = (data1 >> 3) & 1
            self.R2 = (data1 >> 4) & 1
            self.L2 = (data1 >> 5) & 1
            self.F1 = (data1 >> 6) & 1
            self.F3 = (data1 >> 7) & 1

            self.A = (data2 >> 0) & 1
            self.B = (data2 >> 1) & 1
            self.X = (data2 >> 2) & 1
            self.Y = (data2 >> 3) & 1
            self.Up = (data2 >> 4) & 1
            self.Right = (data2 >> 5) & 1
            self.Down = (data2 >> 6) & 1
            self.Left = (data2 >> 7) & 1

        def parse_key(self,data):
            lx_offset = 4
            self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
            rx_offset = 8
            self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
            ry_offset = 12
            self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
            L2_offset = 16
            L2 = struct.unpack('<f', data[L2_offset:L2_offset + 4])[0] # Placeholder，unused
            ly_offset = 20
            self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

        def parse(self,remoteData):
            self.parse_key(remoteData)
            self.parse_botton(remoteData[2], remoteData[3])

        class CustomHandler:
            def __init__(self, stop_event: threading.Event):
                self.remote_controller = UnitreeRemoteController()
                self.stop_event = stop_event

            def init(self):
                self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
                self.lowstate_subscriber.Init(self.lowstate_callback, 10)
            
            def lowstate_callback(self, msg: LowState_):
                self.remote_controller.parse(msg.wireless_remote)

                if self.remote_controller.A == 1:
                    self.stop_event.set()


def main() -> None:
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    sport_client = SportClient()
    sport_client.Init()

    stop_event = threading.Event()
    handler = UnitreeRemoteController.CustomHandler(stop_event)
    handler.init()
    
    vx, vy, vz = 0.3, 0.0, 0.0
    start_time = time.time()

    while time.time() - start_time < 3.0:
        if stop_event.is_set():
            break

        sport_client.Move(vx, vy, vz)
        time.sleep(0.05) 

    stop_event.clear()
    sport_client.StopMove()


if __name__ == "__main__":
    main()