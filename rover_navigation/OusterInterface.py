import ouster.sdk
import ouster.sdk.core
from contextlib import closing
import numpy as np
import cv2
import atexit

from ouster.sdk import sensor
from ouster.sdk import core

#sensor serial 122202003025
#hostname os-122202003025.local

class OusterInterface:
    #hostname = "169.254.180.198/16"#ip address of sesor find and change
   
    



    def __init__(self,debug: bool):
        self.hostname = "os-122202003025"
        self.hostname = "169.254.180.198"
        self.debug = debug
        self.config = core.SensorConfig()
        self.config.udp_profile_lidar = core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16 #find and change?
        self.config.udp_port_lidar = 7502#find and change?
        self.config.operating_mode = core.OperatingMode.NORMAL
        sensor.set_config(self.hostname, self.config, persist=True, udp_dest_auto = True)
        if(debug):
            atexit.register(self.closedug)

    def scan(self):
        with closing(sensor.SensorScanSource(self.hostname, lidar_port=self.config.udp_port_lidar)) as stream:
            for scan, *_ in stream:
                if scan is None:
                    continue
                metadata = stream.sensor_info[0]
                # uncomment if you'd like to see frame id printed
                # print("frame id: {} ".format(scan.frame_id))
                scannedfield = scan.field(core.ChanField.RANGE)
                destaggeredfield = core.data.destagger(metadata,scannedfield)
                xyzlut = core.XYZLut(metadata)
                xyz = xyzlut(destaggeredfield)
                print(self.debug)
                if(self.debug):
                    reflectivity = core.destagger(stream.sensor_info[0],
                                      scan.field(core.ChanField.REFLECTIVITY))
                    reflectivity = (reflectivity / np.max(reflectivity) * 255).astype(np.uint8)
                    cv2.imshow("scaled reflectivity", reflectivity)
                    key = cv2.waitKey(1) & 0xFF
                xyz = xyz.reshape(-1, 3)*1/1000
                return xyz
            
    def closedug(self):
        cv2.destroyAllWindows()
# oi = OusterInterface(True)
# x = True
# while x == True:
#     oi.scan()
    