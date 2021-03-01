import pytest
from src.elliptecstage import ElloHostCommands, ElloStage, ElloDeviceResponses


class TestClass(object):
    def test_mm_to_pulse_8byte_hex_str(self):
        value = 4.0  # [mm]
        value_hex = ElloStage.mm_to_pulse_8byte_hex_str(value)
        assert value_hex == '00002000'

    def testpuls_8byte_hex_str_to_mm(self):
        value_hex = '00001000'  # [8byte hex string]
        value = ElloStage.puls_8byte_hex_str_to_mm(value_hex)
        assert value == 2.0

    def test_conversion(self):
        value_hex = '00001234'  # [8byte hex string]
        value = ElloStage.puls_8byte_hex_str_to_mm(value_hex)
        value_hex2 = ElloStage.mm_to_pulse_8byte_hex_str(value)
        assert value_hex == value_hex2

    def test_compose_command(self):
        cmd = ElloHostCommands(ElloHostCommands._HOSTREQ_MOVEABSOLUTE)  # 'ma'
        value = 4.0  # [mm]
        value_hex = ElloStage.mm_to_pulse_8byte_hex_str(value)
        msg = cmd.compose_command(value_hex)

        assert msg == '0ma00002000'

    def test_parse_message_invalid_input(self):
        msg = ''
        command, data, address = ElloDeviceResponses.parse_message(msg)
        assert ElloDeviceResponses._DEVGET_INVALID == command

    def test_parse_message_valid_input(self):
        msg = b'0PO00002000'
        command, data, address = ElloDeviceResponses.parse_message(msg)
        assert (ElloDeviceResponses._DEVGET_POSITION,
                4.0, 0) == (command, data, address)
