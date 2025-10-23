import os

def setup_uart(device):
    os.system(f"stty -F {device} 115200 cs8 -cstopb -parenb -echo")
    print(f"UART configured on {device}")

def setup_can(interface):
    os.system(f"sudo ip link set {interface} type can bitrate 1000000")
    os.system(f"sudo ip link set up {interface}")
    print(f"CAN interface {interface} is up")
