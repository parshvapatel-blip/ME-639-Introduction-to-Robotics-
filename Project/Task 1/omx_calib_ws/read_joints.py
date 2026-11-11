import math, time
from dynamixel_sdk import PortHandler, PacketHandler

port, pk = PortHandler('/dev/ttyUSB0'), PacketHandler(2.0)
port.openPort(); port.setBaudRate(1000000)

# --- 1. close the gripper (ID 15) and keep it closed ---
G = 15
pk.write1ByteTxRx(port, G, 64, 0)        # torque off to set mode
pk.write1ByteTxRx(port, G, 11, 5)        # current-based position mode
pk.write1ByteTxRx(port, G, 64, 1)        # torque on
pk.write2ByteTxRx(port, G, 102, 200)     # gentle grip current (same as ROBOTIS)
pk.write4ByteTxRx(port, G, 112, 20)      # slow
pk.write4ByteTxRx(port, G, 116, 3126)    # closed on your unit    # closed  (= -0.010 m)
print('Closing gripper...'); time.sleep(2.0)

# --- 2. arm joints torque OFF -> move by hand.  HOLD THE ARM! ---
input('Hold the arm, then press Enter to release joints 11-14...')
for i in (11, 12, 13, 14):
    pk.write1ByteTxRx(port, i, 64, 0)

try:
    while True:
        q = []
        for i in (11, 12, 13, 14):
            t, _, _ = pk.read4ByteTxRx(port, i, 132)   # present position
            t = t - (1 << 32) if t > 2**31 else t
            q.append((t - 2048) * 2 * math.pi / 4096)
        print('joints [rad]: ' + '  '.join(f'{v:+.3f}' for v in q) +
              f'   | joint1 = {math.degrees(q[0]):+.1f} deg   ', end='\r')
        time.sleep(0.1)
except KeyboardInterrupt:
    pk.write1ByteTxRx(port, G, 64, 0)    # release gripper on exit
    port.closePort()
    print()
