#!/usr/bin/python3
import rospy

from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO

from onrobot_rg2_driver.srv import GripperState, GripperStateResponse


class Rg2DriverNode:
    def __init__(self) -> None:
        rospy.init_node("rg2_driver_node", anonymous=False)

        self._joint_state_msg = JointState()
        self._joint_state_msg.header.frame_id = "rg2_base_link"
        self._joint_state_msg.name = ["rg2_gripper_joint"]
        
        self._joint_publisher = rospy.Publisher(
            "/joint_states", JointState, queue_size=1
        )

        self._set_gripp_state_service = rospy.Service(
            "rg2/set_gripper_width", GripperState, self._set_grip_state
        )
        self._set_io_service = rospy.ServiceProxy(
            "/ur_hardware_interface/set_io", SetIO
        )

    def _set_grip_state(self, req):
        rospy.wait_for_service("/ur_hardware_interface/set_io")
        self._set_io_service(fun=3, pin=0, state=req.state)

        rospy.wait_for_service("/ur_hardware_interface/set_io")
        self._set_io_service(fun=1, pin=0, state=1.0)

        return GripperStateResponse(True)

    def _pub_joint_state(self):
        self._joint_state_msg.position = [data]
        self._joint_publisher.publish(self._joint_state_msg)


if __name__ == "__main__":
    try:
        rg2_driver_node = Rg2DriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
