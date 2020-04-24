
import struct
import serial
import xdata
from time import sleep
import os   #Check if we are windows or linux


import binascii


def ahrs_highrate():       

    header = b'\x7f\xff'
    identifier = b'\xfe\x00' 
      
    scalefactor = 32767 / 180  #scale factor for pitch, roll, yaw
    scaled_roll = int (xdata.roll * scalefactor)
    scaled_yaw = int (xdata.headingm * scalefactor)
    #print(yaw)
    #print(scaled_yaw)
    scaled_pitch = int (xdata.pitch * scalefactor)
    scaled_alt = int (xdata.altitude * 3.28084 + 5000)
    #print(altitude)
    #print(scaled_alt)
    scaled_vspeed = int (xdata.vspeed * 196.85)
    scaled_vind = int (xdata.airspeed * 16.8781)

    airspeed_rate = 0
    accel_roll_rate = 0
    accel_normal_rate = 0
    

    ahrs_highrate = struct.pack('>2s2shhHHhhhhh', header, identifier, scaled_roll, scaled_pitch, scaled_yaw, scaled_alt, scaled_vspeed, scaled_vind, airspeed_rate, accel_roll_rate, accel_normal_rate)
    
    checksum =  ((sum(ahrs_highrate[2:])) & 0xFF) ^ 0XFF
    
    checksum = struct.pack('B', checksum)

    


    return (ahrs_highrate + checksum)

def ahrs_lowrate():
        
    ahrs_lowrate = b'\x7f\xff\xfd\x00\x05\xa1\x05\xa2\x05\xa3\x00\x00\x00\x64\x00\x2b\x00\x00\x00\x00'
    checksum =  ((sum(ahrs_lowrate[2:])) & 0xFF) ^ 0XFF
    checksum = struct.pack('B', checksum)

    return (ahrs_lowrate + checksum)

def hxr_serial():

    index = 0
 
    #set up serial
    if os.name == 'nt':
        ahrs_serial = serial.Serial('COM39', 115200)     
    
    else:
        ahrs_serial = serial.Serial('/dev/ttyS10', 115200)

    ahrs_serial.write_timeout = 0
    #ahrs_serial2 = serial.Serial('/dev/ttyS14', 115200)
    print('AHRS Output to {}'.format(ahrs_serial.name))

  

#get xplane data
    while True:
    
        ahrs_1 = ahrs_highrate()
    
        if index < 15:
            #print('AHRS_1: {}'.format(binascii.hexlify(ahrs_1)))
            ahrs_serial.write(ahrs_1)
            #ahrs_serial2.write(ahrs_1)
            index += 1
            sleep(0.05)
        

        else:
            ahrs_2 = ahrs_lowrate()
            #print('{} AHRS_2: {}'.format(time.asctime(), binascii.hexlify(ahrs_2)))
            ahrs_serial.write(ahrs_1)
            ahrs_serial.write(ahrs_2)
            #ahrs_serial2.write(ahrs_1)
            #ahrs_serial2.write(ahrs_2)
            index = 0
            sleep(0.05)





if __name__ == "__main__":
    hxr_serial()
    

    