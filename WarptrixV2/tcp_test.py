import socket
import sys 

TCP_IP = sys.argv[1]
TCP_PORT = 1236 

subframes = 12 # number of greyscale values possible other than black. 
X_SIZE = 80 # half number of colums (160), as one byte (high-nibble and low-nibble) controlls two leds
Y_SIZE = 48 # number of rows


mode = bytearray()
mode.extend('mode'.encode('latin-1'))
mode.append(20) #framerate
mode.append(subframes) #subframe nr
mode.append(1) #use both panels, or not.  NOTE: KEEP THIS VALUE 1 UNTIL FURTHER NOTICE!


data_size =  X_SIZE*Y_SIZE

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

s.send(mode)

for k in range(1000000):
    data1 = bytearray()
    data1.extend('start'.encode('latin-1'))
    data1.append(data_size >> 8)
    data1.append(data_size & 0xff)
    _data_size =  X_SIZE*Y_SIZE
    _data1 = [0 for x in range(data_size)]
    for j in range(Y_SIZE):
            for i in range(0,X_SIZE,8):
                _data1[j*X_SIZE+i+0] = 0x01
                _data1[j*X_SIZE+i+1] = 0x23
                _data1[j*X_SIZE+i+2] = 0x45
                _data1[j*X_SIZE+i+3] = 0x67
                _data1[j*X_SIZE+i+4] = 0x89
                _data1[j*X_SIZE+i+5] = 0xAB
                _data1[j*X_SIZE+i+6] = 0xCD
                _data1[j*X_SIZE+i+7] = 0xEF
    [data1.append(a) for a in _data1]
    s.send(data1)
    print("Frame {0} send.".format(k))
exit()
s.close()



