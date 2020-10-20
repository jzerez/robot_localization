#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Quaternion, Vector3, Point, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Odometry
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import random
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from scipy.stats import norm
from visualization_msgs.msg import Marker, MarkerArray
import pdb
import numpy as np

from tf import TransformBroadcaster
from tf import TransformListener

class ParticleFilter():

    def __init__(self):
        # Initialize node and attributes
        rospy.init_node("ParticleFilter")

        # Subscribers
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)
        self.initial_estimate_sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,self.pose_estimate_callback)

        # publishers
        self.all_particles_pub = rospy.Publisher("/visualization_particles", MarkerArray, queue_size=10)
        self.init_particles_pub = rospy.Publisher("/visualization_init",MarkerArray,queue_size=10)
        self.new_particles_pub = rospy.Publisher("/particlecloud",PoseArray,queue_size=10)

        # constants
        self.number_of_particles = 30
        pos_std_dev = 0.25
        ori_std_dev = 25 * math.pi / 180
        self.initial_std_dev = np.array([[pos_std_dev, pos_std_dev, ori_std_dev]]).T
        self.lidar_std_dev = 0.02
        self.resample_threshold = 0.1

        # changing attributes
        self.particles = np.ones([3,self.number_of_particles], dtype=np.float)
        self.weights = np.ones(self.number_of_particles, dtype=np.float)
        self.odom_tf_time = 0
        self.base_tf_time = 0
        self.scan = None
        self.prev_pose = None
        self.delta_pose = None
        self.initial_pose_estimate = None
        self.pose = None

        # helper classes
        self.occupancy_field = OccupancyField()
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        self.transform_helper = TFHelper()

        rospy.loginfo("Initialized")

    def lidar_callback(self,msg):
        # Lidar Subscriber callback function.
        self.scan = msg.ranges;

    def odom_callback(self,msg):
        # Odometry Subscriber callback function.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        pose = -np.array([[x,y,t]]).T
        if self.prev_pose is None:
            self.prev_pose = pose
        self.pose = pose

        self.apply_odom_transform()
        self.plot_particles(self.particles)
        self.transform_helper.send_last_map_to_odom_transform()

    def pose_estimate_callback(self,msg):
        # Rviz pose estimate callback function.
        position = msg.pose.pose.position
        orientation = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        pose = [position.x,position.y,-orientation - math.pi]
        self.initial_pose_estimate = np.array([pose]).T

    def sample_points(self, mean, std):
        # samples a uniform distribution of points
        stds = np.repeat(std, self.number_of_particles, 1)
        return np.random.normal(mean, stds, [3,self.number_of_particles])

    def resample_points(self):
        # Takes the weights of each particle and resamples them according to that weight
        replace = self.weights < self.resample_threshold
        kept_weights = np.multiply(replace,self.weights)
        probs = kept_weights / sum(kept_weights)
        replace_inds = np.arange(self.weights.size)[replace]

        if kept_weights.size == 0:
            rospy.logerr("No particles meet threshold")
        else:
            for i in replace_inds:
                choice = np.random.choice(kept_weights.size, p=probs)
                weight = kept_weights[choice]
                self.particles[:, i] = self.particles[:, choice]
                self.weights[i] = weight

    def apply_odom_transform(self):
        # Takes rotation and translation from odom and transforms the particles accordingly. Also adds a bit of noise
        self.delta_pose = self.pose - self.prev_pose
        self.prev_pose = self.pose

        if abs(self.delta_pose[2]) < 0.5: # this was added to get rid of the orientation wrap around effect
            delta_std = abs(self.delta_pose) * np.transpose([[1.5, 1.5, 0.9]])
            noisy_deltas = self.sample_points(self.delta_pose, delta_std)

            self.particles = self.particles + noisy_deltas

    def calc_prob(self):
        # Reweight particles based on compatibility with laser scan
        scan = self.scan
        particles = self.particles
        for i, p in enumerate(self.particles.T):
            weight_sum = 0
            xs,ys = self.polar_to_cartesian(self.scan, np.radians(range(361)),p[2])
            lidar_x = xs + p[0]
            lidar_y = ys + p[1]

            # Average the probability associated with each LIDAR reading
            for x, y in zip(lidar_x[::2],lidar_y[::2]):
                dist = self.occupancy_field.get_closest_obstacle_distance(x,y)
                prob = norm(0, self.lidar_std_dev).pdf(dist) / (0.4 / self.lidar_std_dev)
                weight_sum += prob**3
            self.weights[i] = weight_sum / len(lidar_x[::2])

    def update_transform(self, pose, target_frame='base_laser_link'):
        # Currently unused, Updates the transform between the map frame and the odom frame
        if((rospy.get_rostime() != self.odom_tf_time and target_frame == 'odom') or (rospy.get_rostime() != self.base_tf_time and target_frame == 'base_laser_link')):
            self.tf_broadcaster.sendTransform((pose[0], pose[1], 0),
                              quaternion_from_euler(0,0,pose[2]+math.pi),
                              rospy.get_rostime(),
                              target_frame,
                              'map')
        if(target_frame == 'odom'):
            self.odom_tf_time = rospy.get_rostime()
        else:
            self.base_tf_time = rospy.get_rostime()


    def polar_to_cartesian(self, rs, thetas, theta_offset):
        # read the function name ok
        np_rs = np.array(rs)
        np_thetas = np.array(thetas)+theta_offset

        xs = np_rs * np.cos(np_thetas)
        ys = np_rs * np.sin(np_thetas)

        return xs, ys

    def plot_particles(self,particles):
        # plots the particles in the pose array particle cloud topic
        pose_array = PoseArray()
        pose_array.header = Header(stamp = rospy.Time.now(),frame_id="map")
        for i,particle in enumerate(particles.T):
            w = self.weights[i] * 10
            nextPose = Pose()
            nextPose.position = Point(x=particle[0],y=particle[1],z=0)
            nextPose.orientation = Quaternion(*quaternion_from_euler(0,0,particle[2]))
            pose_array.poses.append(nextPose)
        self.new_particles_pub.publish(pose_array)

    def calc_avg_particle(self):
        # calculates the weighted average position of the particles
        return np.sum(self.particles * self.weights, axis=1) / sum(self.weights)

    def main(self):
        r = rospy.Rate(5)

        while(not rospy.is_shutdown() and (self.scan is None or self.initial_pose_estimate is None or self.pose is None)):
            rospy.logwarn("One of the necessary components has not yet been initialized")
            r.sleep()

        self.particles = self.sample_points(self.initial_pose_estimate, self.initial_std_dev)
        while(not rospy.is_shutdown()):
            self.calc_prob()
            best_pose = self.calc_avg_particle()
            robot_pose = Pose(position=Point(x=best_pose[0],y=best_pose[1]),orientation=Quaternion(*quaternion_from_euler(0,0,best_pose[2])))
            self.transform_helper.fix_map_to_odom_transform(robot_pose,rospy.Time.now())
            self.resample_points()
            r.sleep()


if __name__ == '__main__':
    PF = ParticleFilter()
    PF.main()
