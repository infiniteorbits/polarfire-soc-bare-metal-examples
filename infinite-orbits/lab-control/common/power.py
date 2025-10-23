import ivi

class PowerSupply:
    def __init__(self, resource):
        self.psu = ivi.rigol.rigolDP932E(resource, prefer_pyvisa=True)

    def set_output(self, channel, state: bool):
        self.psu.outputs[channel].enabled = state

    def measure(self, channel):
        return {
            "voltage": self.psu.outputs[channel].measure("voltage"),
            "current": self.psu.outputs[channel].measure("current")
        }
