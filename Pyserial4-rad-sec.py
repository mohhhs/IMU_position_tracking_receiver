import serial
import csv
# import numpy
# from decimal import Decimal
from Pyserial_functions import serial_ports

# function takes hex string and bits length to return a signed integer
def strhex_int16(hexstr, bits):
    value = int(hexstr, 16)      # Convert to intger
    if value & (1 << (bits-1)):   # True = negative number
        # ex strhex_int16('B',4) B is signed -5 (1011('B')): unsigned 11(1011) - 16 (10000 (1 shifted left by 4 bits))
        value -= 1 << bits

    return value


serial.Serial()
# open microcontroler port, number 3 in array try print(serial_ports()) if a problem
ser = serial.Serial(serial_ports()[3], baudrate= 38400)
print('We are connected to serial port: '+ser.name)
WritenBytes = ser.write('22345AAA21'.encode()) # use encode so it transmit with no errors
print(WritenBytes)


acc_range = 2 # Gs
gyro_range = 250 # DPS

# put into a csv file
with open('imu_data.csv' , mode = 'w') as imu_file:

    imu_writer = csv.writer(imu_file, delimiter=',')
    imu_writer.writerow(['Count', 'Accel.X', 'Accel.Y', 'Accel.Z', 'Gyro.X', 'Gyro.Y', 'Gyro.Z'])

    i_imu = 0
    while i_imu < 1500:
        # Bytes from microcontroler
        readBytes = ser.read(14)  # Should autmate the length, you can set high number and resonable timeout 0.1?
        # put hex in an array
        readBytes_array = []
        for c in readBytes:
            readBytes_array.append(hex(c))

        # collect imu data by convert array uint8_t hex to int16_t hex
        imu_data = []
        imu_converted = [0,0,0,0,0,0,0] # intialized so I can do '=' instead of 'appeand'
        imu_4_dec = [0,0,0,0,0,0,0]
        for i in range(int(len(readBytes)/2)):
            # get rid of 0x, concatinate two hex bytes, zfill keep them alway two digits
            h = readBytes_array[i*2][2:].zfill(2) + readBytes_array[i*2+1][2:].zfill(2)
            imu_data.append(strhex_int16(h, 16))

        # calculate acc and gyro
        imu_converted[0] = imu_data[0] # counts
        imu_converted[1] = (imu_data[1] * acc_range / 32768)  # Accel.x in G
        imu_converted[2] = imu_data[2] * acc_range / 32768  # Accel.y in G
        imu_converted[3] = imu_data[3] * acc_range / 32768  # Accel.z in G
        imu_converted[4] = (imu_data[4] * gyro_range / 32768)* (3.1415 / 180.0)  # gyro.x in rad per second
        imu_converted[5] = (imu_data[5] * gyro_range / 32768)* (3.1415 / 180.0)  # gyro.x in rad per second
        imu_converted[6] = (imu_data[6] * gyro_range / 32768)* (3.1415 / 180.0)  # gyro.x in rad per second

        # limit to 4 decimals
        imu_4_dec[0] = imu_data[0]
        imu_4_dec[1] = float("{0:.4f}".format(imu_converted[1]))
        imu_4_dec[2] = float("{0:.4f}".format(imu_converted[2]))
        imu_4_dec[3] = float("{0:.4f}".format(imu_converted[3]))
        imu_4_dec[4] = float("{0:.4f}".format(imu_converted[4]))
        imu_4_dec[5] = float("{0:.4f}".format(imu_converted[5]))
        imu_4_dec[6] = float("{0:.4f}".format(imu_converted[6]))

        imu_writer.writerow(imu_4_dec)
        i_imu += 1
# # just history of the numbers
# while i < 2:
#     readBytes = ser.read(14)
#     i += 1
#     mohz.append(readBytes)
# print('\n \n')
ser.close()