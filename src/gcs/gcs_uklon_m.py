import sys
import argparse
import time
import struct
import datetime
import socket
import numpy as np

from RF24 import RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
from RF24 import RF24_1MBPS, RF24_250KBPS, RF24_2MBPS
from RF24 import RF24_CRC_16, RF24_CRC_8, RF24_CRC_DISABLED
from RF24 import RF24 as RF24_CLASS
from RF24 import RF24_CRC_DISABLED
from RF24 import RF24_CRC_8
from RF24 import RF24_CRC_16

def bitwise_xor_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


#radio2 = RF24_CLASS(22, 1) # 1
#radio2 = RF24_CLASS(27, 0) # 2
radio2 = RF24_CLASS(13, 11) # 3
#radio2 = RF24_CLASS(26, 10) # 4

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.setblocking(False)
UDPServerSocket.settimeout(0)
UDPServerSocket.bind(("0.0.0.0", 22000))
addresses = []
packets = [0, 0, 0, 0, 0, 0, 0]
count = 0


def generate_logfile_name():
    now = datetime.datetime.utcnow().replace(microsecond=0)
    isostring = now.isoformat()  # string 2021-04-27T23:17:31
    isostring = isostring.replace("-", "")  # string 20210427T23:17:31
    isostring = isostring.replace(":", "")  # string 20210427T231731, oi ?oi iaai
    return "log/testik_gcs-" + isostring + ".bin"

gyro_calib = [0.5174269005847948, -3.421812865497076, -0.24684210526315856]

if __name__ == '__main__':
    static_payload_size = 32

    radio2.begin()

    radio2.openReadingPipe(1, b'\xac\xac\xac\xac\xac')

    radio2.setCRCLength(RF24_CRC_8)
    radio2.setAddressWidth(5)
    radio2.channel = 95

    radio2.setDataRate(RF24_250KBPS)
    radio2.setAutoAck(True)

    radio2.enableDynamicAck()
    if static_payload_size is not None:
        radio2.disableDynamicPayloads()
        radio2.payloadSize = static_payload_size
    else:
        radio2.enableDynamicPayloads()
        radio2.payloadSize = 1

    #radio2.enableAckPayload()
    

    radio2.startListening()
    radio2.printDetails()

    filename = generate_logfile_name()
    f = open(filename, 'wb')
    fname_raw = filename + ".raw"
    fraw = open(fname_raw, 'wb')

    max_int16 = 0x7fff

    while True:

        if count % 1000 == 0:
            print(packets[0], packets[1], packets[2], packets[3], packets[4], packets[5], packets[6])
        count = count + 1

        has_payload, pipe_number = radio2.available_pipe()
        #print(f'has_payload-{has_payload}, pipe_number={pipe_number}')
        
        try:
            bytesAddressPair = UDPServerSocket.recvfrom(2)
            flag = True
            for i in range(len(addresses)):
                if addresses[i][0] == bytesAddressPair[1][0]:
                    addresses[i] = bytesAddressPair[1]
                    flag = False
                    break
            if flag:
                addresses.append(bytesAddressPair[1])        
            clientIP  = "Client IP Address:{}".format(bytesAddressPair[1])
            print(clientIP)
        except BlockingIOError as e:
            pass
        except Exception as e:
            print(str(e))

        if has_payload:
            payload_size = static_payload_size
            if payload_size is None:
                payload_size = radio2.getDynamicPayloadSize()

            data = radio2.read(payload_size)
            #print('got data %s' % data)
            packet = data
            packet_size = len(packet)
            biter = struct.pack("<B", packet_size)
            unix = time.time()
            p_unix = struct.pack("<d", unix)
            record = p_unix + biter + packet
            f.write(record)
            f.flush()

            for address in addresses:
                UDPServerSocket.sendto(data, address)
            
            try:
                if data[0] == 0xF1:
                    packets[0] = packets[0] + 1
                    continue
                    print("==== Пакет IMU ====")
                    unpack_data = struct.unpack("<BHI9hBfH", data[0:32])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time", unpack_data[2])
                    print ("Данные акселерометра", [x* 488.0 / 1000.0 / 1000.0 for x in unpack_data[3:6]])
                    print ("Данные гироскопа", [x* 1750.0 / 100.0 / 1000.0 for x in unpack_data[6:9]])
                    print ("Данные магнитометра", [x/ 1711.0 for x in unpack_data[9:12]])
                    print ("Освещенность", unpack_data[13])   
                    print ("crc", unpack_data[14])
                    new_crc = bitwise_xor_checksum(data[0:30])
                    if new_crc  == unpack_data[14]:
                        print("\nYAHOOOx6\n")
                    else:
                        print(new_crc)
                    #print ("{: 3.3f}, {: 3.3f}, {: 3.3f}, {: 3.3f}, {: 3.3f}, {: 3.3f}".format(*[x/1000 for x in unpack_data[3:9]]))

                elif data[0] == 0xAC:
                    packets[1] = packets[1] + 1
                    continue
                    print("==== Пакет ATGM ====")
                    unpack_data = struct.unpack("<BHI3fBhH", data[0:24])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    print ("широта:", unpack_data[3])
                    print ("долгота:", unpack_data[4])
                    print ("высота:", unpack_data[5])
                    print ("fix:", unpack_data[6])
                    print ("DS_temp:", unpack_data[7] / 10)
                    print ("crc", unpack_data[8])
                    new_crc = bitwise_xor_checksum(data[0:22])
                    if new_crc  == unpack_data[8]:
                        print("\nYAHOOOx5\n")
                    else:
                        print(new_crc)
                
                elif data[0] == 0xAB:
                    packets[2] = packets[2] + 1
                    continue
                    print("==== Пакет NEO6M ====")
                    unpack_data = struct.unpack("<BHI3fBH", data[0:22])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    print ("широта:", unpack_data[3])
                    print ("долгота:", unpack_data[4])
                    print ("высота:", unpack_data[5])
                    print ("fix:", unpack_data[6])
                    print ("crc", unpack_data[7])
                    new_crc = bitwise_xor_checksum(data[0:20])
                    if new_crc  == unpack_data[7]:
                        print("\nYAHOOOx4\n")
                    else:
                        print(new_crc)

                elif data[0] == 0xF3:
                    packets[3] = packets[3] + 1
                    continue
                    print("==== Пакет MICS ====")
                    unpack_data = struct.unpack("<BHI3fIBhH", data[0:28])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    print ("CO", unpack_data[3])
                    print ("NO2", unpack_data[4])
                    print ("NH3", unpack_data[5])
                    print ("давление", unpack_data[6])
                    print ("влажность", unpack_data[7])
                    print ("температура", unpack_data[8])
                    print ("crc", unpack_data[9])
                    new_crc = bitwise_xor_checksum(data[0:26])
                    if new_crc == unpack_data[9]:
                        print("\nYAHOOOx3\n")
                    else:
                        print(new_crc)

                elif data[0] == 0xF2:
                    packets[4] = packets[4] + 1
                    continue
                    print("==== Пакет GY25 ====")
                    unpack_data = struct.unpack("<BHI7hIhH", data[0:29])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    print ("r-p-y", [x/100 for x in unpack_data[3:6]])
                    print ("quatr", [x/10000 for x in unpack_data[6:10]])
                    print ("давление", unpack_data[10])
                    print ("температура", unpack_data[11] / 100)
                    print ("crc", unpack_data[12])
                    new_crc = bitwise_xor_checksum(data[0:27])
                    if new_crc == unpack_data[12]:
                        print("\nYAHOOOx2\n")
                    else:
                        print(new_crc)


                elif data[0] == 0xF4:
                    packets[5] = packets[5] + 1
                    continue
                    print("==== Пакет GY953_IMU ====")
                    unpack_data = struct.unpack("<BHI9h3BH", data[0:30])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    if unpack_data[13] == 0:
                        print ("Данные акселерометра", [x*2.0/max_int16 for x in unpack_data[3:6]])    
                    elif unpack_data[13] == 1:
                        print ("Данные акселерометра", [x*4.0/max_int16 for x in unpack_data[3:6]])
                    elif unpack_data[13] == 2:
                        print ("Данные акселерометра", [x*8.0/max_int16 for x in unpack_data[3:6]])
                    elif unpack_data[13] == 3:
                        print ("Данные акселерометра", [x*16.0/max_int16 for x in unpack_data[3:6]])
                    if unpack_data[14] == 0:
                        print ("Данные гироскопа", [x*250.0/max_int16 for x in unpack_data[6:9]])
                    elif unpack_data[14] == 1:
                        print ("Данные гироскопа", [x*500.0/max_int16 for x in unpack_data[6:9]])
                    elif unpack_data[14] == 2:
                        print ("Данные гироскопа", [x*1000.0/max_int16 for x in unpack_data[6:9]])
                    elif unpack_data[14] == 3:
                        print ("Данные гироскопа", [x*2000.0/max_int16 for x in unpack_data[6:9]])
                    print ("Данные магнитометра", unpack_data[9:12])
                    print ("acc_rn", unpack_data[12])
                    print ("gyr_rn", unpack_data[13])
                    print ("mag_rn", unpack_data[14])
                    print ("crc", unpack_data[15])
                    new_crc = bitwise_xor_checksum(data[0:28])
                    if new_crc == unpack_data[15]:
                        print("\nYAHOOOx1\n")
                    else:
                        print(new_crc)

                elif data[0] == 0xAA:
                    packets[6] = packets[6] + 1
                    continue
                    print("==== Пакет ORG ====")
                    unpack_data = struct.unpack("<2HIhI3hB", data[0:21])
                    print ("flag", unpack_data[0])
                    print ("id", unpack_data[1])
                    print ("time", unpack_data[2])
                    print ("температура", unpack_data[3] / 100)
                    print ("давление", unpack_data[4])
                    print ("accel", [x/1000 for x in unpack_data[5:8]])
                    print ("crc", unpack_data[8])
                    new_crc = bitwise_xor_checksum(data[0:20])
                    if new_crc  == unpack_data[8]:
                        print("\nYAHOOO\n")
                    else:
                        print(new_crc)


                else:
                    pass
                    print('got data %s' % data)
            except Exception as e:
                print(e)
            #print(data[0])
            #print('got data %s' % data)
            fraw.write(data)
        else:
            #print('got no data')
            pass
        #time.sleep(0.01)
