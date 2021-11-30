from serial import Serial

port = Serial('COM69', 115200)

while True:

    try:

        # print('x%02X' % port.read(1).decode())
        print(port.read(1))

    except KeyboardInterrupt:

        break

port.close()


