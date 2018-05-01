
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 

if __name__ == '__main__':
    rospy.init_node('arm_start')

    arm1 = rospy.Publisher('arm_1_joint/command', Float64, queue_size=10)
    arm2 = rospy.Publisher('arm_2_joint/command', Float64, queue_size=10)
    arm3 = rospy.Publisher('arm_3_joint/command', Float64, queue_size=10)
    arm4 = rospy.Publisher('arm_4_joint/command', Float64, queue_size=10)
    arm5 = rospy.Publisher('arm_5_joint/command', Float64, queue_size=10)
    gripper = rospy.Publisher('gripper_1_joint/command', Float64, queue_size=10)
    

    
    rospy.wait_for_message('joint_states', JointState)
    
    for i in range(5):
        arm1.publish(Float64(0.0))
	arm2.publish(Float64(0.0))
	arm3.publish(Float64(-0.3))
        arm4.publish(Float64(-0.65))
        gripper.publish(Float64(0.15))
	rospy.sleep(0.5)
    for i in range(5):
	gripper.publish(Float64(0.0))
	rospy.sleep(0.5)

    for i in range(5):
        arm1.publish(Float64(0.0))
	arm2.publish(Float64(0.0))
	arm3.publish(Float64(0.0))
        arm4.publish(Float64(0.0))
        gripper.publish(Float64(0.0))
	rospy.sleep(0.5)

	

