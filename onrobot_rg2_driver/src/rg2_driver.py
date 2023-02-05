#!/usr/bin/python3
import rospy

from sensor_msgs.msg import JointState
from ur_msgs.msg import IOStates, Analog
from ur_msgs.srv import SetIO

from onrobot_rg2_driver.srv import GripperState, GripperStateResponse, GripperStateRequest


class Rg2DriverNode:
    def __init__(self) -> None:
        rospy.init_node("rg2_driver_node", anonymous=False)

        self._joint_state_msg = JointState()
        self._joint_state_msg.header.frame_id = "rg2_base_link"
        self._joint_state_msg.name = ["rg2_gripper_joint"]

        self._joint_publisher = rospy.Publisher(
            "/joint_states", JointState, queue_size=1
        )

        rospy.Subscriber(
            "/ur_hardware_interface/io_states",
            IOStates,
            self._pub_joint_state,
            queue_size=1,
        )

        self._set_gripp_state_service = rospy.Service(
            "rg2/set_gripper_width", GripperState, self._set_grip_state
        )
        self._set_io_service = rospy.ServiceProxy(
            "/ur_hardware_interface/set_io", SetIO
        )

    def _set_grip_state(self, req: GripperStateRequest) -> GripperStateResponse:
        rospy.wait_for_service("/ur_hardware_interface/set_io")
        self._set_io_service(fun=3, pin=0, state=req.state)

        rospy.wait_for_service("/ur_hardware_interface/set_io")
        self._set_io_service(fun=1, pin=0, state=1.0)

        return GripperStateResponse(True)

    def _pub_joint_state(self, msg: JointState) -> None:
        if msg.analog_out_states[0].domain == Analog.CURRENT:
            analog_state = msg.analog_out_states[1].state
            rg_joint_state = self._analog_to_joint_state(analog_state)

            self._joint_state_msg.header.stamp = rospy.Time.now()
            self._joint_state_msg.position = [rg_joint_state]
            self._joint_publisher.publish(self._joint_state_msg)

    def _analog_to_joint_state(self, analog_state: float) -> float:
        normalized_analog = (analog_state - 0.004) * 62.5
        return self._map(normalized_analog, 0.0, 1.0, -0.44, 1.0)

    @staticmethod
    def _map(x: float, in_from: float, in_to: float, out_from: float, out_to: float) -> float:
        return (x - in_from) / (in_to - in_from) * (out_to - out_from) + out_from

if __name__ == "__main__":
    try:
        rg2_driver_node = Rg2DriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
