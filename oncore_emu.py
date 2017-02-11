# Motorola Oncore emulator
# Copyright 2017 Thomas Petig
#
# Expecting PPS pulse on CTS
#
import time
import serial
import pynmea2
from fcntl import ioctl
from termios import ( TIOCMIWAIT, TIOCM_RNG, TIOCM_DSR, TIOCM_CD, TIOCM_CTS)

use_utc = 1

wait_signals = (TIOCM_RNG | TIOCM_DSR | TIOCM_CD | TIOCM_CTS)

def checksum(array):
    cs = array[2]
    for i in range(3, len(array)-3):
        cs = cs ^ array[i]
    array[-3] = cs


# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB3',
    baudrate=9600
)

ser2 = serial.Serial('/dev/ttyUSB4', 115200, timeout=0.1)

Ea = bytearray([0x40, 0x40, 0x45, 0x61,
0x01, 0x01, 0x07, 0xCE, # 4
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, # 8
0x0B, 0x96, 0x4F, 0x00, # 15
0x01, 0xEE, 0x62, 0x80, # 19
0x00, 0x00, 0x01, 0x00, # 23
0x00, 0x00, 0x01, 0x00, # 27
0x00, 0x00, # 31
0x00, 0x00, # 33
0x00, 0x00, # 35 current DOP
0x00,       # 37 DOP type 0 = PDOP (3d fix)
10, 8,      # 38
0x02, 8, 0xFF, 0x82, # 40
0x04, 8, 0xFF, 0x82,
0x06, 8, 0xFF, 0x82,
0x08, 8, 0xFF, 0x82,
0x0A, 8, 0xFF, 0x82,
0x0C, 8, 0xFF, 0x82,
0x0E, 8, 0xFF, 0x82,
0x10, 8, 0xFF, 0x82,
0x20,
0xDF, 0x0D, 0x0A])

En = bytearray([0x40, 0x40, 0x45, 0x6E,
0x01, # 4
0x00, # 5
0x00, 0xC0, # 6
0x01,              # 8 pps control mode
0x00, 0x00, 0x01,  # 9 pps rate
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, # 12 time to next fire
0x01,        # 19 pulse status
0x01,        # 20 pulse ref
0x00,        # 21 solution status
0x00,        # 22 Time RAIM status
0x00, 0x03,  # 23 time solution accuracy estimate
0x00,        # 25 sawtooth
0x02, 0x00, 0x00, 0x00, 0x01, # 26
0x04, 0x00, 0x00, 0x00, 0x02,
0x06, 0x00, 0x00, 0x00, 0x03,
0x08, 0x00, 0x00, 0x00, 0x04,
0x0A, 0x00, 0x00, 0x00, 0x05,
0x0C, 0x00, 0x00, 0x00, 0x06,
0x0E, 0x00, 0x00, 0x00, 0x00,
0x10, 0x00, 0x00, 0x00, 0x00,
0xFA, 0x0D, 0x0A])
Bb = bytearray([0x40, 0x40, 0x42, 0x62,
10,
0x02, 0x00, 0x10, 0x50, 0x00, 0x10, 0x00,
0x04, 0x00, 0x00, 0x30, 0x00, 0x20, 0x00,
0x06, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
0x08, 0x00, 0x00, 0x5A, 0x00, 0x00, 0x00,
0x0A, 0x00, 0x00, 0x5A, 0x00, 0x01, 0x00,
0x0C, 0x00, 0x00, 0x5A, 0x00, 0x02, 0x00,
0x0E, 0x00, 0x00, 0x5A, 0x00, 0x03, 0x00,
0x10, 0x00, 0x00, 0x5A, 0x00, 0x04, 0x00,
0x11, 0x00, 0x00, 0x20, 0x00, 0x05, 0x00,
0x12, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x3C, 0x0D, 0x0A])

Ap = bytearray([0x40, 0x40, 0x41, 0x70, 0x32, 0x61, 0x52, 0x99, 0x00, 0x81,
        0x01, 0x2A, 0x0F, 0x54, 0xEB, 0x8B, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x38, 0x0D, 0x0A])

Aw = bytearray([0x40, 0x40, 0x41, 0x77,
0x01, # 0 = GPS time, 1 = UTC time
0x36, 0x0D, 0x0A])

Ag = bytearray([0x40, 0x40, 0x41, 0x67, 0x0A, 0x2C, 0x0D, 0x0A])

At = bytearray([0x40, 0x40, 0x41, 0x74,
0x01,
0x34, 0x0D, 0x0A])

Az = bytearray([0x40, 0x40, 0x41, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x0D, 0x0A])

Bj = bytearray([0x40, 0x40, 0x42, 0x6A, 0x00, 0x28, 0x0D, 0x0A])

Bo = bytearray([0x40, 0x40, 0x42, 0x6F, 0x00, 0x2D, 0x0D, 0x0A])

Ec = bytearray(['@', '@', 'E', 'c',
1,
1,
8,
0,
10,
2,
4,
6,
8,
10,
12,
14,
16,
0x00,
0x0D,
0x0A])

ser.isOpen()

secs = 0;

while True:
    ioctl(ser.fd, TIOCMIWAIT, wait_signals)


    if ser.getCTS() == False:
        if use_utc:
            utc = time.gmtime()
            Ea[4] = utc[1]
            Ea[5] = utc[2]
            Ea[6] = (0xff00 & utc[0]) >> 8
            Ea[7] = 0xff & utc[0]
            Ea[8] = utc[3]
            Ea[9] = utc[4]
            Ea[10] = utc[5]
        else:
            Ea[10] += 1
            if Ea[10] >= 60:
                Ea[10] = 0
                Ea[9] += 1
                if Ea[9] >= 60:
                    Ea[9] = 0
                    Ea[8] += 1
                    if Ea[8] >= 24:
                        Ea[8] = 0

        print Ea[4], Ea[5], Ea[8], Ea[9], Ea[10]

        En[12] = utc[1]
        En[13] = utc[2]
        En[14] = (0xff00 & utc[0]) >> 8
        En[15] = 0xff & utc[0]
        En[16] = utc[3]
        En[17] = utc[4]
        En[18] = utc[5]+1

        secs = secs + 1
        checksum(Ea)
        checksum(En)
        time.sleep(0.0742)
        ser.write(Ea)
        ser.write(En)
        if secs % 2 == 0:
            checksum(Bb)
            ser.write(Bb)
        if secs % 5 == 0:
            checksum(Ap)
            ser.write(Ap)
        if secs % 30 == 0:
            checksum(Aw)
            ser.write(Aw)
        if secs % 21 == 0:
            checksum(Ag)
            ser.write(Ag)
        if secs % 22 == 0:
            checksum(At)
            ser.write(At)
        if secs % 23 == 0:
            checksum(Az)
            ser.write(Az)
        if secs % 24 == 0:
            checksum(Bj)
            ser.write(Bj)
        if secs % 25 == 0:
            checksum(Bo)
            ser.write(Bo)

        line = ser2.readline()
        while len(line) > 0:
            msg = pynmea2.parse(line)
            print str(msg)
            if isinstance(msg, pynmea2.types.talker.GGA):
                lat = int(324000000/90*(float(msg.lat)/100))
                Ea[15] = 0xFF & (lat >> 24)
                Ea[16] = 0xFF & (lat >> 16)
                Ea[17] = 0xFF & (lat >> 8)
                Ea[18] = 0xFF & lat
                lon = int(648000000/180*(float(msg.lon)/100))
                Ea[19] = 0xFF & (lon >> 24)
                Ea[20] = 0xFF & (lon >> 16)
                Ea[21] = 0xFF & (lon >> 8)
                Ea[22] = 0xFF & lon
            if isinstance(msg, pynmea2.types.talker.GSV) and int(msg.msg_num)<4:
                Bb[4] = min(int(msg.num_sv_in_view),12)
                index = (int(msg.msg_num)-1) * 4 * 7
                Bb[5+index] = int(msg.sv_prn_num_1)
                Bb[5+index+7] = int(msg.sv_prn_num_2)
                Bb[5+index+14] = int(msg.sv_prn_num_3)
                Bb[5+index+21] = int(msg.sv_prn_num_4)
                Bb[5+index+3] = int(msg.elevation_deg_1)
                Bb[5+index+7+3] = int(msg.elevation_deg_2)
                Bb[5+index+14+3] = int(msg.elevation_deg_3)
                Bb[5+index+21+3] = int(msg.elevation_deg_4)
                Bb[5+index+4] = 0xFF & (int(msg.azimuth_1)>>8)
                Bb[5+index+5] = 0xFF & int(msg.azimuth_1)
                Bb[5+index+7+4] = 0xFF & (int(msg.azimuth_2)>>8)
                Bb[5+index+7+5] = 0xFF & int(msg.azimuth_2)
                Bb[5+index+14+4] = 0xFF & (int(msg.azimuth_3)>>8)
                Bb[5+index+14+5] = 0xFF & int(msg.azimuth_3)
                Bb[5+index+21+4] = 0xFF & (int(msg.azimuth_4)>>8)
                Bb[5+index+21+5] = 0xFF & int(msg.azimuth_4)

                Ea[38] = min(int(msg.num_sv_in_view),12)
                Ea[39] = min(int(msg.num_sv_in_view),8)
                if int(msg.msg_num) <3:
                    index = (int(msg.msg_num)-1) * 4 * 4
                    Ea[40+index] = int(msg.sv_prn_num_1)
                    Ea[40+index+4] = int(msg.sv_prn_num_2)
                    Ea[40+index+8] = int(msg.sv_prn_num_3)
                    Ea[40+index+12] = int(msg.sv_prn_num_4)
                    if msg.snr_1 == '':
                        msg.snr_1 = 0
                    Ea[40+index+2] = int(msg.snr_1)
                    if msg.snr_2 == '':
                        msg.snr_2 = 0
                    Ea[40+index+4+2] = int(msg.snr_2)
                    if msg.snr_3 == '':
                        msg.snr_3 = 0
                    Ea[40+index+8+2] = int(msg.snr_3)
                    if msg.snr_4 == '':
                        msg.snr_4 = 0
                    Ea[40+index+12+2] = int(msg.snr_4)
                    index = (int(msg.msg_num)-1) * 4 * 5
                    En[26+index] = int(msg.sv_prn_num_1)
                    En[26+5+index] = int(msg.sv_prn_num_2)
                    En[26+10+index] = int(msg.sv_prn_num_3)
                    En[26+15+index] = int(msg.sv_prn_num_4)
            line = ser2.readline()




#        checksum(Ec)
#        ser.write(Ec)
        #print Ea
        print ''.join(format(x, '02x') for x in Ea)
        print ''.join(format(x, '02x') for x in En)
        print ''.join(format(x, '02x') for x in Bb)

