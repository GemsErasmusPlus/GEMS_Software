#socat -d -d pty,link=/tmp/ttyRX,raw,echo=0 pty,link=/tmp/ttyTX,raw,echo=0

import serial
import time
import math

SERIAL_PORT = '/tmp/ttyTX'
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def simulate_transmission():
    count = 0
    while True:
        data = f"{count};{math.sin(math.radians(count*0.2))};{math.sin(math.radians(count*0.1))}\n"
        ser.write(data.encode('utf-8'))
        count += 1
        time.sleep(0.001)  # Delay to simulate periodic data sending

if __name__ == "__main__":
    try:
        print("START")
        simulate_transmission()
    except KeyboardInterrupt:
        print("END")
    finally:
        ser.close()
