import sys
import argparse
import time
import struct
import datetime

from RF24 import RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
from RF24 import RF24_1MBPS, RF24_250KBPS, RF24_2MBPS
from RF24 import RF24_CRC_16, RF24_CRC_8, RF24_CRC_DISABLED
from RF24 import RF24 as RF24_CLASS
from RF24 import RF24_CRC_DISABLED
from RF24 import RF24_CRC_8
from RF24 import RF24_CRC_16


radio2=RF24_CLASS(24, 1)
#radio2=RF24_CLASS(22, 0)


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


    if static_payload_size is not None:
        radio2.disableDynamicPayloads()
        radio2.payloadSize = static_payload_size
    else:
        radio2.enableDynamicPayloads()

    #radio2.enableAckPayload()
    #radio2.enableDynamicAck()

    radio2.startListening()
    radio2.printDetails()

    filename = generate_logfile_name()
    f = open(filename, 'wb')
    fname_raw = filename + ".raw"
    fraw = open(fname_raw, 'wb')
    while True:
        has_payload, pipe_number = radio2.available_pipe()
        #print(f'has_payload-{has_payload}, pipe_number={pipe_number}')

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

            try:
                if data[0] == 0xF1:
                    pass
                    print("==== Пакет IMU ====")
                    unpack_data = struct.unpack("<BHI9hH", data[0:27])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time", unpack_data[2])
                    print ("Данные акселерометра", [x/1000 for x in unpack_data[3:6]])
                    print ("Данные гироскопа", [x/1000 for x in unpack_data[6:9]])
                    print ("Данные магнитометра", [x/1000 for x in unpack_data[9:12]])
                    print ("crc", unpack_data[12])
                    #summ[0] += ([x/1000 for x in unpack_data[9:12]])[0]
                    #summ[1] += ([x/1000 for x in unpack_data[9:12]])[1]
                    #summ[2] += ([x/1000 for x in unpack_data[9:12]])[2]
                    #count += 1
                    #print([x/count for x in summ])
                elif data[0] == 0xAC:
                    pass
                    print("==== Пакет ATGM ====")
                    unpack_data = struct.unpack("<BHI3fBhH", data[0:24])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    print ("широта:", unpack_data[3])
                    print ("долгота:", unpack_data[4])
                    print ("высота:", unpack_data[5])
                    print ("fix:", unpack_data[6])
                    print ("DS_temp:", unpack_data[7])
                    print ("crc", unpack_data[8])
                
                elif data[0] == 0xAB:
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

                elif data[0] == 0xF3:
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

                elif data[0] == 0xF2:
                    print("==== Пакет GY25 ====")
                    unpack_data = struct.unpack("<BHI3fIhH", data[0:27])
                    print ("flag", unpack_data[0])
                    print ("num", unpack_data[1])
                    print ("time:", unpack_data[2])
                    print ("roll", unpack_data[3])
                    print ("yaw", unpack_data[4])
                    print ("pitch", unpack_data[5])
                    print ("давление", unpack_data[6])
                    print ("температура", unpack_data[7] / 100)
                    print ("crc", unpack_data[8])

                elif data[0] == 0xAA:
                    print("==== Пакет ORG ====")
                    unpack_data = struct.unpack("<2HIhI3hB", data[0:21])
                    print ("flag", unpack_data[0])
                    print ("id", unpack_data[1])
                    print ("time", unpack_data[2])
                    print ("температура", unpack_data[3] / 100)
                    print ("давление", unpack_data[4])
                    print ("accel", [x/1000 for x in unpack_data[5:8]])
                    print ("crc", unpack_data[8])

                else:
                    print('got data %s' % data)
            except Exception as e:
                print(e)
            #print(data[0])
            #print('got data %s' % data)
            fraw.write(data)
            fraw.flush()
        else:
            #print('got no data')
            pass
        time.sleep(0.1)
