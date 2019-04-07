#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
from scipy.spatial import KDTree


#Test MZ
STATE_COUNT_THRESHOLD = 3
IMAGE_PROCESS_RATE = 5

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoint_tree = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        # self.use_simulator = rospy.get_param('~simulator_used')
        self.use_simulator = not self.config['is_site']
        
        # rospy.logwarn('use simulator {0}'.format(self.use_simulator))
        self.light_classifier = TLClassifier(self.use_simulator)

        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.image_process_count = 0

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        if not self.waypoint_tree:
            base_waypoint_array = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                   waypoints.waypoints]
            self.waypoint_tree = KDTree(base_waypoint_array)



    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        # Process every IMAGE_PROCESS_RATE images (if not using simulator, process every image)
        if self.image_process_count == 0 or not self.use_simulator:
            light_wp, state = self.process_traffic_lights()
        else:
            light_wp = self.last_wp
            state = self.state

        self.image_process_count = (self.image_process_count + 1)%IMAGE_PROCESS_RATE

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state  #if the state changes, update the state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            # after the state occurs `STATE_COUNT_THRESHOLD` number, if the light is red, update the index of
            # the waypoint closest to the red light's stop line to traffic waypoint, else set the index to -1
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            # rospy.logdebug('publishing ' + str(light_wp))
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            # rospy.logdebug('publishing es ' + str(self.last_wp))
            self.upcoming_red_light_pub.publish(Int32(self.last_wp)) #publish the last index of the waypoint
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            UNKNOWN=4 GREEN=2 YELLOW=1 RED=0

        """
        # For testing, just return the light state
        #return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification, return the state
        return self.light_classifier.get_classification(cv_image)
        # return TrafficLight.GREEN

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        line_wp_idx = None
        closest_upcoming_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoint_tree):
            car_wp_inx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            diff = len(self.waypoints.waypoints) # here we set the diff as large as possible
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                tmp_wp_idx = self.get_closest_waypoint(stop_line_positions[i][0], 
                                                        stop_line_positions[i][1])

                # Find the closest stop line waypoint index
                d = tmp_wp_idx - car_wp_inx
                if d >= 0 and d < diff:
                    diff = d
                    closest_upcoming_light = light
                    line_wp_idx = tmp_wp_idx

        if closest_upcoming_light:
            state = self.get_light_state(closest_upcoming_light)
          
            return line_wp_idx, state
        else:
            return -1, TrafficLight.UNKNOWN
        

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
