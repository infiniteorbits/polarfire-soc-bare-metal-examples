import time
from scapy.all import *
#enx0011226846b7
IF = "enx207bd255f302"  
MAC = "00:fc:00:12:34:56"      


packet = Ether(dst=MAC) / Raw(load=b'X' * 512)

print(f"TX")
sendp(packet, iface=IF, loop=1, verbose=0)
