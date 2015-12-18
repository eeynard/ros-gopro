import socket
import sys
from time import sleep


def get_command_msg(id):
    return "_GPHD_:%u:%u:%d:%1lf\n" % (0, 0, 2, 0)


UDP_IP = "10.5.5.9"
UDP_PORT = 8554
KEEP_ALIVE_PERIOD = 2500
KEEP_ALIVE_CMD = 2
MESSAGE = get_command_msg(KEEP_ALIVE_CMD)

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)
print("message:", MESSAGE)

if sys.version_info.major >= 3:
    MESSAGE = bytes(MESSAGE, "utf-8")

while True:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    sleep(KEEP_ALIVE_PERIOD / 1000)
