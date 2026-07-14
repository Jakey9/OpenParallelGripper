import sys
import time

from xarm.wrapper import XArmAPI


def int_to_2bytes_list(value):
    value_in_hex = hex(value)[2:].zfill(4)
    return [int(value_in_hex[0:2], base=16), int(value_in_hex[2:4], base=16)]


class ParalleGripperAtom:
    def __init__(self, arm, torque_limit=150):
        self._arm = arm
        self._arm.set_mode(0)
        self._arm.set_state(0)
        code = self._arm.set_tgpio_modbus_baudrate(115200)
        print('set_tgpio_modbus_baudrate, code={}'.format(code))
        # self.close_in_res = 200
        # self.open_in_res = 2200
        self.close_in_res = 1918
        self.open_in_res = 839

    @property
    def pos_min(self):
        return min(self.close_in_res, self.open_in_res)

    @property
    def pos_max(self):
        return max(self.close_in_res, self.open_in_res)

    def open(self):
        data = [0x01, 0x06, 0x00, 0x80] + int_to_2bytes_list(self.open_in_res)
        self._arm.getset_tgpio_modbus_data(data)
        time.sleep(1)

    def close(self):
        data = [0x01, 0x06, 0x00, 0x80] + int_to_2bytes_list(self.close_in_res)
        self._arm.getset_tgpio_modbus_data(data)
        time.sleep(1)

    def move(self, pos):
        assert self.pos_min <= pos <= self.pos_max

        data = [0x01, 0x06, 0x00, 0x80] + int_to_2bytes_list(pos)
        self._arm.getset_tgpio_modbus_data(data)
        time.sleep(1)

    def move_out_of_10(self, ratio):
        assert 0 <= ratio <= 9
        pos = self.open_in_res + (self.close_in_res - self.open_in_res) * ratio / 9
        self.move(int(pos))

    def get_pos(self):
        data = [0x01, 0x03, 0x01, 0x01, 0x00, 0x02]
        res = self._arm.getset_tgpio_modbus_data(data)
        return res


if __name__ == "__main__":
    action_list = "oc"
    ip = sys.argv[1]
    if len(sys.argv) >= 3:
        action_list = sys.argv[2]
    arm = XArmAPI(ip, is_radian=False)
    pga = ParalleGripperAtom(arm)

    for action in action_list:
        if action == "o":
            pga.open()

        elif action == "c":
            pga.close()

        elif action == "s":
            time.sleep(1)

        elif action.isdecimal():
            val = int(action)
            pga.move_out_of_10(val)
