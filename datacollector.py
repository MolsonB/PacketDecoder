#!/usr/bin/python3
import socket
import time
import sys
from decode import Decode
from signal import signal, SIGINT
import xdata
import message

import xplane
import os                       # Check if we are windows or linux
import crcmod.predefined        # CRC16.X25
import binascii
from threading import Timer
import ctypes

EFIS_UDP_PORT = 10001                   # Do not chang
EFIS_UDP_TIMEOUT = 30                   # How long to wait for EFIS to check in
EFIS_IPADDRESS = "192.168.0.5"          # EFIS IPAddress (hardcode, only for debugging to save time)
MY_LINK_IPADDRESS = 0x10                # We are interlink ID 16 (dec)


# Keep track of a dictionary of counters for debugging
Counters = {}
Counters["HelloTx"] = 0
Counters["HelloRx"] = 0
Counters["PacketFeed"] = 0
Counters["PacketRxGood"] = 0
Counters["PacketRxBad"] = 0
Counters["PacketTxGood"] = 0
Counters["PacketTxBad"] = 0




# Setup class to break out GPS bits from uint32_T
c_uint32 = ctypes.c_uint32   
class DateTimeBits(ctypes.LittleEndianStructure):
    _fields_ = [
        ("month", c_uint32, 4), 
        ("day", c_uint32, 5),
        ("hour", c_uint32, 5),
        ("min", c_uint32, 6),
        ("sec", c_uint32, 6),
        ("status",c_uint32, 1)
    ] 
class GPSDateTimes(ctypes.Union):
    _anonymous_ = ("bit",)
    _fields_ = [
        ("bit", DateTimeBits),
        ("asByte", c_uint32)
    ]



class DataCollector():

    def __init__(self):
        self.is_running = True

        self.efis_state_variables = {}      # State variables stored on EFIS
        
        # UDP socket
        self.efis_ipaddress = ''             # List of EFIS IPaddresses connected
        self.sock_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    #socket.IPPROTO_UDP
        self.sock_udp.setblocking(False)        # non-blocking, allows UDP and TCP recv
        print('Listening for EFIS pings on UDP port {}...'.format(EFIS_UDP_PORT))
        self.sock_udp.bind(('', EFIS_UDP_PORT))
        self.timer_udp = Timer(EFIS_UDP_TIMEOUT, self.timer_udp_set, '0')
        self.timer_udp_set('1')

        # TCP socket
        self.tcp_send_time = 0                # Low style timer for sending data
        self.tcp_send_index = 0               # Index to loop through sending data
        self.tcp_buffer = bytearray()
        self.sock_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_tcp.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if os.name != 'nt':   # Does not work in windows 7 right now TODO
            self.sock_tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
            self.sock_tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)
        self.sock_tcp.bind(('', EFIS_UDP_PORT))
        self.sock_tcp.listen(1)
   

    def handler(self, signal_received, frame):
        # Handle any cleanup here
        self.connection.close()
        self.connection = None
        self.sock_tcp = None
        print('SIGINT or CTRL-C detected. Exiting gracefully')
        sys.exit(0)


    # Setup timer to monitor EFIS status over UDP
    def timer_udp_set(self, task):
        if task == '1':
            if self.timer_udp.is_alive():
                self.timer_udp.cancel()
  
            self.timer_udp = Timer(EFIS_UDP_TIMEOUT, self.timer_udp_set, '0')          # should get an UDP packet in 10 seconds
            self.timer_udp.start()
        else:
            print("UDP timed out")
            self.is_running = False
            sys.exit("How to restart the Timer to check for EFIS ??? TODO")


    # EFIS expects a ping (Hello) every 10 seconds
    def send_hello(self, udp = False):
        
        print("Packet   RX Good:{} RX Bad:{}   TX Good:{} TX Bad:{}".format(Counters["PacketRxGood"], Counters["PacketRxBad"], Counters["PacketTxGood"], Counters["PacketTxBad"]))
        Counters["HelloTx"] += 1
#        print("Sending Hello to EFIS ({})".format(Counters["HelloTx"]))

        payload = bytearray()
        payload.append(0x00)                # packet type 00 = Hello
        payload.append(0x01)                # link version 
        payload.extend((0x00, 0x00))        # display serial number

        if udp:
            self.send_data(payload, udp)     # Send over UDP
        else:
            self.send_data(payload)          # Send over TCP

    
    # Pack payload with header and checksum, send to EFIS
    def send_data(self, payload, UDP = False):
        packet = bytearray()
        packet.append(0x5B)                 # vendor protocol code
        packet.append(MY_LINK_IPADDRESS)    # source ID
        packet.append(0xFF)                 # broadcast to all IPs
        packet.append(0x0A)                 # Time To Live 
        packet.extend(payload)

        # Add Checksum crc16.x25
        crc16 = crcmod.predefined.Crc('x-25')
        crc16.update(packet)
        packet.extend(crc16.crcValue.to_bytes(2, 'little'))         

        packet = packet.replace(b'\x7D',b'\x7D\x5D')        # Stuff Byte (Do this first)
        packet = packet.replace(b'\x7E',b'\x7D\x5E')        

        if UDP:
            try:
                self.sock_udp.sendto(packet, (self.efis_ipaddress, EFIS_UDP_PORT))
            except:
                print('Error sending UDP data to EFIS')

        else:
            # TCP needs FrameFlags where UDP does not
            packet.insert(0, 0x7E)
            packet.append(0x7E)
            try: 
                self.connection.sendall(packet)
                #self.sock_tcp.sendall(packet)
                Counters["PacketTxGood"] += 1
            except:
                print('Error sending TCP data to EFIS')
                Counters["PacketTxBad"] += 1

        return packet  

       
            

       

    def stop(self):
        self.is_running = False
    
    
    # Listen on the multicast UDP for EFIS pings (Hello) - Only once then close
    def listen_udp(self):
        """The UDP hello is a broadcast for units to find each other and establish a TCP connection between each unit. 
        When the UDP broadcasts are exchanged, the unit with the lower IP address of the pair initiates the TCP connection. 
        The EFIS will accept a connection in any order, though. 
        You can skip sending UDP broadcasts and initiate a TCP connection as soon as you see a UDP hello if you don't want to manage 
        accepting TCP connections and sending UDP broadcasts.
        """
        try:
            buffer = self.sock_udp.recvfrom(12)
        except:
            # Cheating by setting the ipaddress so we don't have to wait for udp packet
            if not self.efis_ipaddress and 'EFIS_IPADDRESS' in globals():
                buffer = ['bypass',(EFIS_IPADDRESS,'port')]
            else:
                return

        if buffer:
            self.timer_udp_set('1')     # reset timer

            message = buffer[0]
            address = buffer[1]
            ipaddress = address[0]

            if not self.efis_ipaddress:
                self.efis_ipaddress = ipaddress
                print('EFIS says Hello from ipaddress: {}'.format(self.efis_ipaddress))
                self.send_hello(True)        # Send Hello back on UDP to setup TCP handshake on EFIS

                # Start TCP once 
                self.connection, self.client_address = self.sock_tcp.accept()
                self.connection.settimeout(0.1)


    def timeout_udp(self):
        print('UDP timed out')
        self.is_running = False    


    # Listen on TCP and decode/verify the packets
    def listen_tcp(self):
        Counters["PacketFeed"] += 1
        try:
            self.tcp_buffer += self.connection.recv(1024) 
        except:
            return  # timeout, no data

        header = -1

        while True:       
            header = self.tcp_buffer.find(b'\x7E')               # Find first instance of Frame flag (could be starting or ending)
                  
            if header >= 0:
                end = self.tcp_buffer.find(b'\x7E',header+1)      # Look for the ending Frameflag, 2 bytes past the start so we know
                                                                 # it's the end flag, and not the start of a new packet
                if end >= 2:      
                    #print("Feed {}: FrameFlags{}".format(Counters["PacketFeed"], self.tcp_buffer.count(b'\x7E')))
                    #print("Org {}".format(binascii.hexlify(self.tcp_buffer)))               

                    packet = self.tcp_buffer[header+1:end]           # Grab 1st packet out of buffer and removes frame flags at same time
                    del self.tcp_buffer[:end+1]                      # Resize buffer to end of 1st packet
                    #print("Packet {}".format(binascii.hexlify(packet)))             
                    #print("Remain {}".format(binascii.hexlify(self.tcp_buffer)))  

                    packet = packet.replace(b'\x7D\x5E',b'\x7E')        # Stuff Byte (Do this one first)
                    packet = packet.replace(b'\x7D\x5D',b'\x7D')        
                    msglen = len(packet)
                    check_sum = int.from_bytes(packet[msglen-2:msglen], "little")  # last two bytes are checksum, grab the range
                    crc16 = crcmod.predefined.Crc('x-25')
                    crc16.update(packet[0:msglen-2])
                    
                    if check_sum == crc16.crcValue:
                        Counters["PacketRxGood"] += 1
                        yield packet[4:msglen-2]        # remove headers and checksum
                    else:
                        Counters["PacketRxBad"] += 1
                        #print("Bad Checksum")

                    Counters["PacketFeed"] = 0
                else:
                    #print("End Frame flag not found yet")
                    break
            else:
                #print("Start Frame flag not found yet")
                break 


    # Process the packet
    def process_packet(self, packet):
        """The header has been stripped out of the payload already
        vendorcode = msg[0];       0x5B    vendor protocol code
        scr = msg[1];              0x0A    source ID
        dest = msg[2];             0xFF    broadcast to all
        ttl = msg[3];              0x0A    Time To Live
        """

        type = packet[0]
        payload = packet[1:len(packet)]

        # Hello
        if type == 0x00:           
            Counters["HelloRx"] += 1
            msg = "Receiving Hello from EFIS ({})".format(Counters["HelloRx"])
            # Send Hello back, use their packet as a timer
            self.send_hello()  
        
        # State variables
        elif type == 0x02:          
            print("Received Efis state variables")
            for var in payload.split(b'\x00'):
                if var:
                   var = var.split(b'=')
                   self.state_varibles(int(var[0].decode()), var[1].decode())

        #elif type == 0x07
            #07005C00       #When I cleared the 'Check altitude' message box


        # GPS
        elif type == 0x09:
            subtype = payload[0]
            if subtype== 0x00:           # time, date, position, speed, mag var, from current GPS source, like data in GPRMC
                """Byte 0: Type = 0 (GPS position packet)
                Byte 1: Last two digits of year (year mod 100), zero if unknown
                Byte 2-5: packed LSB first: month (4 bits) | day (5 bits) | hour (5 bits) | min (6 bits) | sec (6 bits) | status (1 bit)
                    The date and time fields are all zeroes if unknown.
                    The status bit is 1 when the GPS has indicated its data is valid.
                    The EFIS may try to use this data as a time source if the day is non-zero and status is 1.
                Byte 6-9: latitude (32-bit float)
                Byte 10-13: longitude (32-bit float)
                Byte 14-15: Ground track in tenths of a degree, MSB first
                Byte 16-17: Magnetic variation in hundredths of a degree, MSB first, positive west
                Byte 18-19: Ground speed in tenths of a knot, MSB first
                Byte 20: bit field
                    Bit 0 = GPS2 input is configured on this unit
                    Bit 1 = This data is from GPS2
                """
                year = 2000 + payload[1]  
                (var,) = struct.unpack('I', payload[2:6]) 
                datetime = GPSDateTimes()
                datetime.asByte = var
                if datetime.status == 0:
                    print("GPS datetime is invalid")  
                
                (latitude,) = struct.unpack('f', payload[6:10])    
                (longitude,) = struct.unpack('f', payload[10:14])
                (ground_track,) = struct.unpack(">H", payload[14:16])    
                ground_track /= 10
                (mag_variation,) = struct.unpack('>h', payload[16:18])
                mag_variation /= 100
                (ground_speed,) = struct.unpack('>H', payload[18:20])
                ground_speed /= 10
     #           msg = "valid:{} = {}-{}-{} {}:{}:{} Lat:{} Long:{} Trk:{} MagVar:{} GS:{}".format(datetime.status, year, datetime.month, datetime.day, datetime.hour, datetime.min, datetime.sec, latitude, longitude, groundTrack, magVariation, groundSpeed)
     #           print(msg)
 
            elif subtype == 0x01:          # navigation data to active waypoint, like data in GPRMB
                """eg1. $GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*0B
                       A            Data status A = OK, V = warning
                       0.66,L       Cross-track error (nautical miles, 9.9 max.),
                                            steer Left to correct (or R = right)
                       003          Origin waypoint ID
                       004          Destination waypoint ID
                       4917.24,N    Destination waypoint latitude 49 deg. 17.24 min. N
                       12309.57,W   Destination waypoint longitude 123 deg. 09.57 min. W
                       001.3        Range to destination, nautical miles
                       052.5        True bearing to destination
                       000.5        Velocity towards destination, knots
                       V            Arrival alarm  A = arrived, V = not arrived
                       *0B          mandatory checksum
                """
                print(binascii.hexlify(payload)) 
                print(binascii.hexlify(payload[1:3]),end='')
                (dest_latitude,) = struct.unpack('f', payload[3:7])    
                (dest_longitude,) = struct.unpack('f', payload[7:11])
                (orig_latitude,) = struct.unpack('f', payload[11:15])    
                (orig_longitude,) = struct.unpack('f', payload[15:19])
                (true_bearing,) = struct.unpack('>H', payload[19:21])    # 3 degree higher then on efis
                (destination_range,) = struct.unpack('>H', payload[21:23])   # in nM    why2
                print(binascii.hexlify(payload[23:29]))
                #payload[24]
                #payload[25]
                #payload[26]
                #payload[27]
                #payload[28]
                (destination_range2,) = struct.unpack('>H', payload[29:31])   #in nM   why2
                destination_waypoint = payload[31:]    #ascii     
                destination_range /= 10

            elif subtype == 0x02:          # waypoints in active flight plan from current GPS source
                print(binascii.hexlify(payload))  

            elif subtype == 0x03:          # time and date from GPS1 and/or GPS2 independent of current GPS source
                """Byte 0: 03 = time/date
                Byte 1: GPS Source (pretty sure)
                Byte 2: Last two digits of year (year mod 100), zero if unknown
                Byte 3-6: packed LSB first: month (4 bits) | day (5 bits) | hour (5 bits) | min (6 bits) | sec (6 bits) | status (1 bit)
                """
                year = 2000 + payload[2]  
                (var,) = struct.unpack('I', payload[3:7]) 
                datetime = GPSDateTimes()
                datetime.asByte = var
                if datetime.status:
                    msg = "Gps: valid:{} = {}-{}-{} {}:{}:{}".format(datetime.status, year, datetime.month, datetime.day, datetime.hour, datetime.min, datetime.sec)
     #               print(msg)
                else:
                    print("GPS datetime is invalid")

            elif subtype == 0x04:           # GPS altitude and geoidal difference, fix quality, number of satellites used, from current GPS source, like data in GPGGA   */        //Gps position packet
                gps_mode = payload[1]            # Gps Mode      3 = Auto Fix 3D
                gps_source = payload[2]          # Gps source
                sats_in_calculation = payload[3]  # SatInCalculation 
                (gps_altitude,) = struct.unpack('f', payload[4:8])    
                gps_altitude = round(gps_altitude*3.281,1)      # convert meters to ft
                (geoidal,) = struct.unpack('f', payload[8:12])       

                msg = "gpsMode:{} gpsSource:{} satsInCalculation:{} gpsAltitude:{} geoidal:{}".format(gps_mode, gps_source, sats_in_calculation, gps_altitude, geoidal)
       
        elif type == 0x1A:      # 0x1A  Nav/Com state packet. volume levels, audiopanel modes, transponder modes, drive boxes upper right hand corner
            return                       
        else:
            print('Packet {} not setup for processing yet'.format(type))
            print(binascii.hexlify(packet))  


    #Saving the EFIS state varibles, relaying over to X-plane
    def state_varibles(self, index, value):
        """3 = Select Heading bug            NOT SURE  divid by 0.0174532924791086 to get degree
        4 = Selected Altitude
        12 = Baro          Not sure what last two digits are
        18 = Screen dim  (255,128,64,32,16,8,4,2,1,0)
        25 = LAT A/p   0=ENav, 1=Hdg
        35 = DA = Decision Altitude
        36 = Missed App Altitude
        37 = Preset Altitude
        49 = 
        7 = AUTO=1 3 VS=4 3  ASPD=3 4    VNav=
        13 = Climb IAS  (Auto mode, also changed #30)
        14 = VRate
        30 = Climb and descent on a selected IAS airspeed (ASPD mode)
        31 = Climb and descent on vertical Speed Rate (VS mode)
        6 
        93  = SAP
        """

        if index == 3:      # Select Heading bug
            value = round(float(value) / 0.0174532924791086)
#            Xplane.sendDREF("sim/cockpit/autopilot/heading_mag", int(value))
        elif index == 4:    # Selected Altitude
            value = value
#            Xplane.sendDREF("sim/cockpit/autopilot/altitude", int(value))
        elif index == 12:   #Baro pressure        
            ref = "sim/cockpit/misc/barometer_setting"
            xplane.send_data(ref, value)
        elif index == 35 or index == 36 or index == 37:
            if value == -2147483648:    #clear value
                value = ""

        self.efis_state_variables[index] = value
        print("Efis state varible {} = {}".format(index, value))


    # Index loop to send packets to EFIS via TCP
    # TODO find an elegant way to have a Sending thread within this thread
    def data_to_efis(self):
        ndx = self.tcp_send_index
        if time.time() - self.tcp_send_time > 0:
            self.tcp_send_time =  time.time() + 0.2        
        else:
            return

        if ndx == 0:
            self.send_data(message.gps(0))
            ndx += 1
        elif ndx == 1:
            self.send_data(message.gps(3))
            ndx += 1
        elif ndx == 2:
            self.send_data(message.gps(4))
            ndx += 1
        else:
            self.send_data(message.eis())
            ndx = 0
        
        self.tcp_send_index = ndx    


    # Main thread loop
    def run(self):

        while self.is_running:
            self.listen_udp() 

            if self.efis_ipaddress:      # found IP address, now listen to TCP stream
                for packet in self.listen_tcp():
                    self.process_packet(packet)
 
                self.data_to_efis()


            
if __name__ == "__main__":
    # Create and instance of the class using the HXr hostname.
    uc = DataCollector()
    signal(SIGINT, uc.handler)
    print('Running. Press CTRL-C to exit.')
    uc.run()

            