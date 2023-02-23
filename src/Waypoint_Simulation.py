"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String, Header, Bool


class OffbPosCtl:
    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 1
    sim_ctr = 1

    des_pose = PoseStamped()
    isReadyToFly = False

    # Waypoint locations
    locations = numpy.matrix([

                               [41, 3.5, 16, 1, 0, 0, 0],
                               [41, 3.5, 15, 1 , 0 , 0, 0],

                               [55, -12, 19.5, 1, 0, 0, 0],
                               [57.25, -9.75, 19.5, 0.9239, -0.3827, 0, 0],
                               [59.5, -7.5, 19.5, 0.7071 , -0.7071 , 0 ,0 ],
                               [62, -9, 19.25, 0.3827, -0.9239, 0, 0],
                               [66, -12, 19.25, 0, 1, 0, 0],
                               [62, -14.75, 19.25, 0.3827, 0.9239, 0, 0],
                               [59.5, -18, 19.25, 0.7071, 0.7071, 0, 0],
                               [57.75, -14.75, 19, 0.9239, 0.3827, 0, 0],
                               [55, -12, 19, 1, 0, 0, 0],

                              [13,-65,2.5,0.7071,-0.7071,0,0],
                              [13, -65, -5, 0.7071, -0.7071, 0, 0]
                              ])

    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)


        NUM_UAV = 2
        armed=0
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]

        # Comm for drones
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape

        while not rospy.is_shutdown():
            #print self.sim_ctr, shape[0], self.waypointIndex

            if self.waypointIndex == 13 : #Disarming Sequence

                print("Landing & Disarm Initiated")
                success = [None for i in range(NUM_UAV)]
                for uavID in range(0, NUM_UAV):
                    rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

                for uavID in range(0, NUM_UAV):
                    try:
                        success[uavID] = arm_proxy[uavID](False)
                    except rospy.ServiceException, e:
                        print ("mavros1/set_mode service call failed: %s" % e)
                break

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s" % e)



            if armed == 0:
                armed=1
                print("Drone Armed and Taking off")
                success = [None for i in range(NUM_UAV)]
                for uavID in range(0, NUM_UAV):
                    rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

                for uavID in range(0, NUM_UAV):
                    try:
                        success[uavID] = arm_proxy[uavID](True)
                    except rospy.ServiceException, e:
                        print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.waypointIndex == 1 :
                print("Reached Probe Location")
            elif self.waypointIndex == 4 :
                print("Reached Rock Location")

            if self.isReadyToFly:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14/2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg


    def drone_state_cb(self, msg):
        #print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            #print "readyToFly"



if __name__ == "__main__":
    OffbPosCtl()
