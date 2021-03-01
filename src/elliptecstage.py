import serial
from enum import Enum


# 1 # Communication time out
# 2 # Mechanical time out
# 3 # Command error or not supported
# 4 # Value out of range
# 5 # Module isolated
# 6 # Module out of isolation
# 7 # Initializing error
# 8 # Thermal error
# 9 # Busy
# 10 # Sensor Error (May appear during self test. If code persists there is an error)
# 11 # Motor Error (May appear during self test. If code persists there is an error)
# 12 # Out of Range (e.g. _stage has been instructed to move beyond its travel range).
# 13 # Over Current error
# 14 # General error. Applicable only to Motorized Paddle Polarizer


class ElloDeviceUtility(Enum):
    @classmethod
    def position(self, data):
        return ElloStage.puls_8byte_hex_str_to_mm(data)

    @classmethod
    def undefined(self, data):
        return data


class ElloDeviceResponses(tuple, Enum):
    _DEVGET_STATUS = (b'GS', ElloDeviceUtility.undefined)
    _DEVGET_POSITION = (b'PO', ElloDeviceUtility.position)  # GS or PO
    _DEV_GETPOSITION = (b'PO', ElloDeviceUtility.undefined)
    _DEVGET_INFORMATION = (b'IN', ElloDeviceUtility.undefined)
    _DEVGET_MOTOR1INFO = (b'I1', ElloDeviceUtility.undefined)
    _DEVGET_MOTOR2INFO = (b'I2', ElloDeviceUtility.undefined)
    _DEVGET_CURRENTCURVEMEASURE_MOTOR1 = (b'C1', ElloDeviceUtility.undefined)
    _DEVGET_CURRENTCURVEMEASURE_MOTOR2 = (b'C2', ElloDeviceUtility.undefined)
    _DEVGET_HOMEOFFSET = (b'HO', ElloDeviceUtility.undefined)
    _DEVGET_JOGSTEPSIZE = (b'GJ', ElloDeviceUtility.undefined)
    _DEVGET_VELOCITY = (b'GV', ElloDeviceUtility.undefined)
    _DEVGET_BUTTONSTATUS = (b'BS', ElloDeviceUtility.undefined)
    _DEVGET_BUTTONPOSITION = (b'BO', ElloDeviceUtility.undefined)
    _DEVGET_INVALID = (b'', ElloDeviceUtility.undefined)  # invalid command

    def __init__(self, command_id):
        self.command_id = command_id

    def is_same(self, command_id):
        return self.command_id == command_id

    @staticmethod
    def parse_message(msg):
        if len(msg) < 5:  # device reply is at least 5bytes
            return ElloDeviceResponses._DEVGET_INVALID, '', -1

        address = int(chr(msg[0]))
        command_id = msg[1:3]
        data = msg[3:len(msg)]  # truncate /r/n

        # Search for command
        for command in ElloDeviceResponses:
            if command[0] == command_id:
                data = command[1](data.decode())
                return command, data, address

        return ElloDeviceResponses._DEVGET_INVALID, data, address


class ElloHostCommands(Enum):
    _HOSTREQ_STATUS = 'gs'
    # _HOSTREQ_INFORMATION = 'in'_DEV
    _HOSTREQ_SAVE_USER_DATA = 'us'
    _HOSTREQ_CHANGEADDRESS = 'ca'
    _HOSTREQ_MOTOR1INFO = 'i1'
    _HOSTSET_FWP_MOTOR1 = 'f1'
    _HOSTSET_BWP_MOTOR1 = 'b1'
    _HOSTREQ_SEARCHFREQ_MOTOR1 = 's1'
    _HOSTREQ_SCANCURRENTCURVE_MOTOR1 = 'c1'
    _HOST_ISOLATEMINUTES = 'is'
    _HOSTREQ_MOTOR2INFO = 'i2'
    _HOSTSET_FWP_MOTOR2 = 'f2'
    _HOSTSET_BWP_MOTOR2 = 'b2'
    _HOSTREQ_SEARCHFREQ_MOTOR2 = 's2'
    _HOSTREQ_SCANCURRENTCURVE_MOTOR2 = 'c2'
    _HOSTREQ_HOME = 'ho'
    _HOSTREQ_MOVEABSOLUTE = 'ma'
    _HOSTREQ_MOVERELATIVE = 'mr'
    _HOSTREQ_HOMEOFFSET = 'go'
    _HOSTSET_HOMEOFFSET = 'so'
    _HOSTREQ_JOGSTEPSIZE = 'gj'
    _HOSTSET_JOGSTEPSIZE = 'sj'
    _HOST_FORWARD = 'fw'
    _HOST_BACKWARD = 'bw'
    _HOST_MOTIONSTOP = 'ms'
    _HOST_GETPOSITION = 'gp'
    _HOSTREQ_VELOCITY = 'gv'
    _HOSTSET_VELOCITY = 'sv'
    _HOST_GROUPADDRESS = 'ga'

    def __init__(self, command_id):
        self.command_id = command_id

    def compose_command(self, hex_ascii_string='', address_string='0'):
        return address_string + self.command_id + hex_ascii_string


class ElloStage:
    _PULS_PER_MM = 2048

    # Initialize the connection with the motor
    def __init__(self, port='COM3', n=0):
        # start serial on _stage
        self._stage = serial.Serial(port=port, baudrate=9600, timeout=0.2)
        self._n = n
        self._n_str = str(n)
        self.initialize_motor()

    # Convert a position to 8bytes hex string in upper cases
    # Ello devices are case sensitive
    @classmethod
    def mm_to_pulse_8byte_hex_str(self, pos):
        hex_str = format(round(pos * 2048), '08X')
        return hex_str

    @classmethod
    def puls_8byte_hex_str_to_mm(self, hex_str):
        pos = int(hex_str, 16)/self._PULS_PER_MM
        return pos

    def read_message(self):
        msg = self._stage.readline()
        msg = msg.strip()
        return msg

    def read_message_blocking(self,
                              trigger_command=ElloDeviceResponses._DEVGET_STATUS,
                              timeout_trial=5):
        if timeout_trial < 0:
            timeout_trial = 0
        count = 0
        while True:
            msg = self.read_message()
            command, data, address = ElloDeviceResponses.parse_message(msg)

            # Check if the device response is desirable one
            if command is trigger_command:
                return command, data, address

            count = count+1
            if count > timeout_trial:
                return ElloDeviceResponses._DEVGET_INVALID, data, address

    def read_message_blocking_position_response(self):
        return self.read_message_blocking(trigger_command=ElloDeviceResponses._DEVGET_POSITION)

    def send_command(self, ello_command, value=''):
        command = ello_command.compose_command(value)
        self._stage.write(command.encode())

    def raw_command(self, raw_command):
        self._stage.write(raw_command.encode())

    def initialize_motor(self):
        self.send_command(ElloHostCommands._HOSTSET_FWP_MOTOR1, '00B7')
        self.send_command(ElloHostCommands._HOSTSET_BWP_MOTOR1, '008D')
        self.send_command(ElloHostCommands._HOSTSET_FWP_MOTOR2, '00B3')
        self.send_command(ElloHostCommands._HOSTSET_BWP_MOTOR2, '008B')
        self.send_command(ElloHostCommands._HOSTSET_VELOCITY, '64')

    def get_motor1info(self):
        self.send_command(ElloHostCommands._HOSTREQ_MOTOR1INFO)

    def get_motor2info(self):
        self.send_command(ElloHostCommands._HOSTREQ_MOTOR2INFO)

    def move_absolute(self, pos):
        value = self.mm_to_puls_8byte_hex_str(pos)
        self.send_command(ElloHostCommands._HOSTREQ_MOVEABSOLUTE, value)

    def move_relative(self, pos):
        value = self.mm_to_puls_8byte_hex_str(pos)
        self.send_command(ElloHostCommands._HOSTREQ_MOVERELATIVE, value)

    def move_home(self):
        direction = '0'  # consistency for rotary stages
        self.send_command(ElloHostCommands._HOSTREQ_HOME, direction)


if __name__ == "__main__":
    test()
