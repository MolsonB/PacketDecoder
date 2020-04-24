import struct
import datetime
import xdata
import ctypes
import binascii

'''
    UDP listens for a HELLO packet from HXr
    Format:
        HELLO
        --> 7e frame flag
        --> 5b vendor protocol code
        --> 01 source=1 (Primary ID)
        --> ff destination=255 (broadcast)
        --> 0a TTL=10
        --> 00 packet type=0 (hello)
        --> 01 link version=1
        --> 00 serial number=1
        --> 01 (LSB of serial number)
        --> 94 checksum=0x7f94
        --> 7f (MSB of checksum)
        --> 7e frame flag


    protocol = data[1]
    source = data[2]
    destination = data[3]
    ttl = data[4]
    packet_type = data[5]
    payload = data[6:-3]
    checksum = data[-3:-1].hex()

'''

#Setup class to break out GPS bits from uint32_T
c_uint32 = ctypes.c_uint32   
class DateTime_bits( ctypes.LittleEndianStructure ):
    _fields_ = [
        ("month", c_uint32, 4 ), 
        ("day",   c_uint32, 5 ),
        ("hour",  c_uint32, 5 ),
        ("min",   c_uint32, 6 ),
        ("sec",   c_uint32, 6 ),
        ("status", c_uint32, 1 )
    ] 
class GPSDateTimes( ctypes.Union ):
    _anonymous_ = ("bit",)
    _fields_ = [
        ("bit",    DateTime_bits ),
        ("asByte", c_uint32    )
    ]


#Xplane GPS to EFIS interlink
def gps(task): 
    
    payload = bytearray()
    payload.append(0x09)        #Packet Type
    
    if task == 0:               #time, date, position, speed, mag var, from current GPS source, like data in GPRMC
        timeNow = datetime.datetime.now()
        year = str(timeNow.year)
        year = int(year[2:])
        timeBits = GPSDateTimes()
        timeBits.bit.month = timeNow.month
        timeBits.bit.day = timeNow.day
        timeBits.bit.hour = timeNow.hour
        timeBits.bit.min = timeNow.minute
        timeBits.bit.sec = timeNow.second
        timeBits.bit.status = 1

        track = int(xdata.headingt*10)
        magvar = int(xdata.magvar*100) 
        gndspeed = int(xdata.gndspeed*10*1.94384)
        bits = 0

        payload.append(0x00)                     #Byte 0: 00 = GPS position packet
        payload.append(year)                     #Byte 1: Last two digits of year (year mod 100), zero if unknown
        payload.extend(struct.pack('I', timeBits.asByte))          #Byte 2-5: packed LSB first: month (4 bits) | day (5 bits) | hour (5 bits) | min (6 bits) | sec (6 bits) | status (1 bit)
                                                    #The date and time fields are all zeroes if unknown.
                                                    #The status bit is 1 when the GPS has indicated its data is valid.
                                                    #The EFIS may try to use this data as a time source if the day is non-zero and status is 1.
        payload.extend(struct.pack('ff', xdata.latitude, xdata.longitude))   #Byte 6-9: latitude (32-bit float)
                                                                        #Byte 10-13: longitude (32-bit float)
        payload.extend(struct.pack(">HhHb", track, magvar, gndspeed, bits))  #Byte 14-15: Ground track in tenths of a degree, MSB first
                                                                        #Byte 16-17: Magnetic variation in hundredths of a degree, MSB first, positive west
                                                                        #Byte 18-19: Ground speed in tenths of a knot, MSB first
                                                                        #Byte 20: bit field
                                                                            #Bit 0 = GPS2 input is configured on this unit
                                                                            #Bit 1 = This data is from GPS2

    elif task == 3:             #time and date from GPS1 and/or GPS2 independent of current GPS source
        timeNow = datetime.datetime.now()
        year = str(timeNow.year)
        year = int(year[2:])
        timeBits = GPSDateTimes()
        timeBits.bit.month = timeNow.month
        timeBits.bit.day = timeNow.day
        timeBits.bit.hour = timeNow.hour
        timeBits.bit.min = timeNow.minute
        timeBits.bit.sec = timeNow.second
        timeBits.bit.status = 1

        payload.append(0x03)                #Byte 0: 03 = time/date
        payload.append(0x00)                #Byte 1: GPS Source (pretty sure)
        payload.append(year)                #Byte 2: Last two digits of year (year mod 100), zero if unknown
        payload.extend(struct.pack('I', timeBits.asByte))          #Byte 3-6: packed LSB first: month (4 bits) | day (5 bits) | hour (5 bits) | min (6 bits) | sec (6 bits) | status (1 bit)
                                                    #The date and time fields are all zeroes if unknown.
                                                    #The status bit is 1 when the GPS has indicated its data is valid.
                                                    #The EFIS may try to use this data as a time source if the day is non-zero and status is 1.

    elif task == 4:             #GPS altitude and geoidal difference, fix quality, number of satellites used, from current GPS source, like data in GPGGA   */        //Gps position packet
        gpsAltitude = int(xdata.altitude)   #in meters
        geoidal = 0

        payload.append(0x04)    
        payload.append(0x03)        #Gps Mode      3 = Auto Fix 3D
        payload.append(0x00)        #Gps source
        payload.append(0x05)        #SatInCalculation 
        payload.extend(struct.pack('ff', gpsAltitude, geoidal))    


    return payload


#Xplane Engine data to EFIS
def eis():
    payload = bytearray()
    payload.append(0x0F)            #Packet Type = EIS1 0x0F    EIS2 0x27
    
    #convert and scale variables
    cht = [0] * 6
    egt = [0] * 9
    aux = [0] * 6

    rpm = int(xdata.rpm)
    if rpm < 0:
        rpm = 0
    cht[0:4] = (int(xdata.cht),) * 4    #wrap scalar in an iterable
    egt[0:4] = (int(xdata.egt),) * 4
    airspeed = 0        #not displayed in EFIS
    altimeter = 0       #not displayed in EFIS
    volts = float(xdata.volts)
    fuelflow = float(xdata.fuelflow * 1286.33)      #convert kg_sec to gal_hour xx.x
    internaltemp = 0        #Don't think is used in EFIS
    manifoldtemp = -100        #aka carb temperature
    verticalspeed = 0       #Not sure if used in EFIS
    oat = int(xdata.oat)
    oiltemp = int(xdata.oiltemp)
    oilpressure = int(xdata.oilpressure)
    aux[0] = int(xdata.manifoldpressure * 10)
    aux[1] = int(xdata.fuelpressure * 10)
    aux[2] = 0
    aux[3] = 0
    aux[4] = 0
    aux[5] = 0
    coolanttemp = 0
    hobbs = float(xdata.hobbs / 3600)
    fuelqty = float((xdata.fuel_qty_left + xdata.fuel_qty_right) / 2.72155)         #gallon is 2.72155kg (6lbs)
    flight_hrs = (int(xdata.flighttime / 3600))         #HH:MM:SS
    flight_min = (int((xdata.flighttime % 3600) / 60))
    flight_sec = (int((xdata.flighttime % 3600) % 60))
    fuelflowtime = 0                        #Fuel Flow Time until empty HH:MM
    baropressure = xdata.baropressure       #Not sure if being used
    savebit = 0                 #The bits are set when we see 10 zero values in a row
                                #Bit0 = tachometer has stopped (steady at zero)   
                                #Bit1 = fuel flow has stopped (steady at zero)
                                #Bit2 = additional data follows for a CAN bus input
    rpm2 = 0
    eisver = 59             #0x00 0x3B

    payload.extend(struct.pack(">H6H9HH", rpm, cht[0], cht[1], cht[2], cht[3], cht[4], cht[5], egt[0], egt[1], egt[2], egt[3], egt[4], egt[5], egt[6], egt[7], egt[8], airspeed))
    payload.extend(struct.pack("<fff", altimeter, volts, fuelflow))
    payload.extend(struct.pack(">bbfhHB", internaltemp, manifoldtemp, verticalspeed, oat, oiltemp, oilpressure))
    payload.extend(struct.pack(">hhhhhhH", aux[0], aux[1], aux[2], aux[3], aux[4], aux[5], coolanttemp))
    payload.extend(struct.pack("<ff", hobbs, fuelqty))
    payload.extend(struct.pack(">BBBH", flight_hrs, flight_min, flight_sec, fuelflowtime))
    payload.extend(struct.pack("<f", baropressure))
    payload.extend(struct.pack(">BHH", savebit, rpm2, eisver))

#    print(binascii.hexlify(payload))

    return payload

if __name__ == "__main__":
   
    eis()






 

            

        