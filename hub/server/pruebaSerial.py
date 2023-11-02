import struct
import sys

def main():

    data=(struct.pack('d',2) + struct.pack('d',2) + struct.pack('d',2 ) 
    + struct.pack('d', 1) + struct.pack('d', 1) + struct.pack('d', 1))
    len = 48
    cabecera = struct.pack('H',112) + struct.pack('H',22)+ struct.pack('H', 9) + struct.pack('H',48)
    message=cabecera + data
    '''data= (struct.pack('H', 112) + struct.pack('H', 22) + struct.pack('H', 9) + 
    struct.pack('H', 48) + struct.pack('d',2) + struct.pack('d',2) + struct.pack('d',2 ) 
    + struct.pack('d', 1) + struct.pack('d', 1) + struct.pack('d', 1) )'''

    print(message)
    print(struct.unpack('HHHHdddddd',message))


if __name__ == "__main__":
    main()