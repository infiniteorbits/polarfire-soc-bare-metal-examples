import sys
from common.power import PowerSupply
from common.uart import setup_uart
from common.can import setup_can
import yaml

with open("config/devices.yaml") as f:
    CONFIG = yaml.safe_load(f)["L50"]

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 L50.py [set_up UART|set_up CAN|power ON|power OFF]")
        sys.exit(1)

    action = sys.argv[1].lower()

    if action == "set_up":
        target = sys.argv[2].lower()
        if target == "uart":
            setup_uart(CONFIG["uart"])
        elif target == "can":
            setup_can(CONFIG["can"])
    elif action == "power":
        target = sys.argv[2].lower()
        psu = PowerSupply(CONFIG["power_resource"])
        psu.set_output(CONFIG["power_channel"], target == "on")
        print(psu.measure(CONFIG["power_channel"]))
    else:
        print("Unknown command")

if __name__ == "__main__":
    main()
