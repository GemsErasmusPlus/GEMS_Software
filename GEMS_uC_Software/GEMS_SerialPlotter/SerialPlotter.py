import serial
from collections import deque
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
BUFFER_SIZE = 10000

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

cyclic_buffer = deque([(0.0, 0.0, 0.0)] * BUFFER_SIZE, maxlen=BUFFER_SIZE)

buffer_lock = threading.Lock()

def parse_data(line):
    try:
        data = line.strip().split(';')
        if len(data) == 3:
            return float(data[0]), float(data[1]), float(data[2])
    except ValueError:
        pass
    return None

def read_serial():
    while True:
        line = ser.readline().decode('utf-8')
        if line:
            data = parse_data(line)
            if data:
                with buffer_lock:
                    cyclic_buffer.append(data)

def update_plot(frame):
    with buffer_lock:
        data = list(cyclic_buffer)
    if data:
        t, a, b = zip(*data)
        line_a.set_data(t,a)
        line_b.set_data(t,b)
        ax.relim()
        ax.autoscale_view()
    return line_a, line_b

if __name__ == "__main__":
    thread = threading.Thread(target=read_serial)
    thread.daemon = True
    thread.start()

    # Plot setup
    fig, ax = plt.subplots(figsize=(15, 3))
    line_a, = ax.plot([], [], 'b-')
    line_b, = ax.plot([], [], 'r-')
    plt.ylim(0, 1)

    ani = FuncAnimation(fig, update_plot, interval=100, blit=False)

    plt.show()
    ser.close()
