#!/usr/bin/python3
import rospy

from sensor_msgs.msg import JointState

from onrobot_rg2_driver.srv import GripperState, GripperStateResponse, GripperStateRequest

from rg2_modbus import Rg2Modbus


class Rg2DriverNode:
    def __init__(self) -> None:
        rospy.init_node("rg2_driver_node", anonymous=False)

        ur_robot_ip = rospy.get_param('~ur_robot_ip', '10.42.0.49')
        frame_id = rospy.get_param('~base_link_frame', 'rg2_base_link')
        joint_name = rospy.get_param('~gripper_joint_name', 'rg2_gripper_joint')
        self.service_call_timeout_sec = rospy.get_param('~service_call_timeout_sec', 10.0)

        self._rg2_modbus = Rg2Modbus(ur_robot_ip)

        self._joint_state_msg = JointState()
        self._joint_state_msg.header.frame_id = frame_id
        self._joint_state_msg.name = [joint_name]

        self._joint_publisher = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self._set_gripp_state_service = rospy.Service(
            "rg2/set_gripper_width", GripperState, self._set_grip_state
        )

        rospy.Timer(rospy.Duration(0.1), self._pub_joint_state_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _set_grip_state(self, req: GripperStateRequest) -> GripperStateResponse:
        rospy.loginfo(
            f'[{rospy.get_name()}] Sending gripp request:\n'
            f'\t - width: {req.width}\n\t - force: {req.force}'
        )

        start_time = rospy.Time.now()
        self._rg2_modbus.send_gripp_req(req.width, req.force)

        while not self._rg2_modbus.querry_rg_busy_flag():
            rospy.sleep(0.05)

            if (rospy.Time.now() - start_time).secs > self.service_call_timeout_sec:
                rospy.logwarn(f'[{rospy.get_name()}] Grip action failed: timeout') 
                return GripperStateResponse(False, False, 'Grip action failed: timeout')

        rospy.loginfo(f'[{rospy.get_name()}] Request send succesful.')

        while self._rg2_modbus.querry_rg_busy_flag():
            rospy.sleep(0.1)

            if (rospy.Time.now() - start_time).secs > self.service_call_timeout_sec:
                rospy.logwarn(f'[{rospy.get_name()}] Grip action failed: timeout')
                return GripperStateResponse(False, False, 'Grip action failed: timeout')
        
        grip_detected = self._rg2_modbus.querry_grip_detected_flag()
        success = True
        
        rospy.loginfo(f'[{rospy.get_name()}] Grip action succes!')

        return GripperStateResponse(success, grip_detected, '')

    def _pub_joint_state_cb(self, *args) -> None:
        self._joint_state_msg.header.stamp = rospy.Time.now()

        rg_width = self._rg2_modbus.querry_gripper_width()
        joint_state = self._map(rg_width, 0.0, 101.0, -0.44, 1.0)
        self._joint_state_msg.position = [joint_state]

        self._joint_publisher.publish(self._joint_state_msg)

    @staticmethod
    def _map(x: float, in_from: float, in_to: float, out_from: float, out_to: float) -> float:
        return (x - in_from) / (in_to - in_from) * (out_to - out_from) + out_from


if __name__ == "__main__":
    try:
        rg2_driver_node = Rg2DriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
