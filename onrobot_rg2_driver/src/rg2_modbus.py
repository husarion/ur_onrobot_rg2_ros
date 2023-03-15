from dataclasses import dataclass
import modbus_tk.defines as cst
from modbus_tk import modbus_tcp
from time import sleep
from typing import Tuple


@dataclass
class ColisConsts:
    WIDTH_OUT = 16
    RG_BUSY_OUT = 17
    GRIP_DETECTED_OUT = 18
    ACTION_TRIGGER_IN = 20
    WIDTH_IN = 128
    FORCE_IN = 129


class Rg2Modbus:
    def __init__(self, ur_robot_ip: str) -> None:
        self._master = modbus_tcp.TcpMaster(host=ur_robot_ip, port=502)
        self._master.set_timeout(5.0)

    def send_gripp_req(self, width: float, force: float) -> None:
        # set gripper width and force
        self._master.execute(
            1,
            cst.WRITE_MULTIPLE_REGISTERS,
            ColisConsts.WIDTH_IN,
            output_value=[int(width * 10), int(force * 10)],
        )

        # action trigger
        self._master.execute(
            1, cst.WRITE_SINGLE_COIL, ColisConsts.ACTION_TRIGGER_IN, output_value=1
        )
        sleep(0.1)
        self._master.execute(
            1, cst.WRITE_SINGLE_COIL, ColisConsts.ACTION_TRIGGER_IN, output_value=0
        )

    def querry_rg_busy_flag(self) -> bool:
        return self._master.execute(1, cst.READ_COILS, ColisConsts.RG_BUSY_OUT, 1)[0]

    def querry_grip_detected_flag(self) -> bool:
        return self._master.execute(1, cst.READ_COILS, ColisConsts.GRIP_DETECTED_OUT, 1)[0]

    def querry_gripper_width(self):
        return self._master.execute(1, cst.READ_HOLDING_REGISTERS, ColisConsts.WIDTH_OUT, 1)[0] / 10
