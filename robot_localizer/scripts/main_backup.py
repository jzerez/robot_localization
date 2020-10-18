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

class ParticleFilter():

    def __init__(self):
        #
        rospy.init_node("ParticleFilter")
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)
        self.all_particles_pub = rospy.Publisher("/visualization_particles", MarkerArray, queue_size=10)
        self.init_particles_pub = rospy.Publisher("/visualization_init",MarkerArray,queue_size=10)
        self.initial_estimate_sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,self.pose_estimate_callback)

        self.occupancy_field = OccupancyField()
        self.number_of_particles = 20
        self.particles = np.ones([3,self.number_of_particles], dtype=np.float)
        self.weights = np.ones(self.number_of_particles, dtype=np.float)
        pos_std_dev = 0.25
        ori_std_dev = 25 * math.pi / 180
        self.initial_std_dev = np.array([[pos_std_dev, pos_std_dev, ori_std_dev]]).T

        self.lidar_std_dev = 0.05
        self.resample_threshold = 0.2
        self.scan = None
        self.prev_pose = None
        self.delta_pose = None
        self.initial_pose_estimate = None
        self.pose = None

        rospy.loginfo("Initialized")

    def sample_points(self, mean, std):
        # samples a uniform distribution of points
        stds = np.repeat(std, self.number_of_particles, 1)
        self.particles = np.random.normal(mean, stds,
                                          [3,self.number_of_particles])


    def resample_points(self):
        # Takes the weights of each particle and resamples them according to that weight
        replace_inds = self.weights < self.resample_threshold
        kept_weights = self.weights[~replace_idns]
        probs = kept_weights / sum(kept_weights)

        if sum(replace_inds) == 0:
            rospy.logerr("No weights")
        else:
            for i in replace_inds:
                choice = np.random.choice(kept_weights.size, p=probs)
                weight = kept_weights[choice]
                self.particles[:, i] = self.particles[:, choice]
                self.weights[i] = weight

    def apply_odom_transform(self):
        # Takes rotation and translation from odom and transforms the particles accordingly. Also adds a bit of noise
        self.delta_pose = self.pose - self.prev_pose
        delta_std = abs(self.delta_pose) * np.transpose([[1.5, 1.5, 0.7]])
        noisy_deltas = self.sample_points(self.delta_pose, delta_std)

        self.particles += noisy_deltas

        self.prev_pose = self.pose

    def lidar_callback(self,msg):
        # Lidar Subscriber callback function.
        self.scan = msg.ranges;

    def odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        pose = -np.array([[x,y,t]]).T
        if self.prev_pose == None:
            self.prev_pose = pose
            self.pose = pose
        else:
            self.pose = pose

        self.apply_odom_transform()
        self.plot_particles(self.particles, ColorRGBA(1, 0, 0.5, 0.5), self.all_particles_pub)

    def pose_estimate_callback(self,msg):
        rospy.logdebug("Callback Good")
        position = msg.pose.pose.position
        orientation = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        pose = [position.x,position.y,orientation - math.pi]
        self.initial_pose_estimate = np.array([pose]).T

    def calc_prob(self):
        # Reweight particles based on compatibility with laser scan

        for i, p in enumerate(self.particles.T):
            weight_sum = 0
            xs,ys = self.polar_to_cartesian(self.scan, np.radians(range(361)),p[2])
            lidar_x = xs + p[0]
            lidar_y = ys + p[1]

            # Average the probability associated with each LIDAR reading
            for x, y in zip(lidar_x,lidar_y):
                dist = self.occupancy_field.get_closest_obstacle_distance(x,y)
                prob = norm(0, self.lidar_std_dev).pdf(dist) / (0.4 / self.lidar_std_dev)
                weight_sum += prob
            self.weights[i] = weight_sum / 361

    def update_transform(self, pose, target_frame='base_laser_link'):
        # Updates the transform between the map frame and the odom frame
        br = TransformBroadcaster()
        br.sendTransform((pose[0], pose[1], 0),
                          quaternion_from_euler(0, 0, pose[2]+math.pi),
                          rospy.get_rostime(),
                          target_frame,
                          'map')


    def polar_to_cartesian(self, rs, thetas, theta_offset):
        # read the function name ok
        np_rs = np.array(rs)
        np_thetas = np.array(thetas)+theta_offset

        xs = np_rs * np.cos(np_thetas)
        ys = np_rs * np.sin(np_thetas)

        return xs, ys

    def plot_particles(self, particles, color, pub):
        marker_array = MarkerArray()
        for i, particle in enumerate(particles):
            nextMarker = Marker()
            x = particle[0]
            y = particle[1]
            t = particle[2]
            nextMarker.header = Header(stamp = rospy.Time.now(), frame_id="map")
            nextMarker.id = i;
            nextMarker.ns="particle"
            nextMarker.type = Marker.ARROW
            nextMarker.points = [Point(x,y,0), Point(x+math.cos(t), y+math.sin(t), 0)]
            # nextMarker.pose = Pose(Point(x,y,0), Quaternion(0,0,0,0))
            nextMarker.scale = Vector3(0.1,0.3,0.2)
            nextMarker.color = color
            nextMarker.action = Marker.ADD
            marker_array.markers.append(nextMarker)
        pub.publish(marker_array)

    def main(self):
        r = rospy.Rate(5)

        while(not rospy.is_shutdown() and (self.scan == None or self.initial_pose_estimate == None or self.pose == None)):
            rospy.logwarn("Still Missing Something")
            r.sleep()


        # TODO: get inital pose estimate from RVIZ
        self.sample_points(self.initial_pose_estimate, self.initial_std_dev)
        counter = 0
        while(not rospy.is_shutdown()):

            max_weight = max(self.weights)
            best_pose = self.particles[self.weights.index(max_weight)]
            self.update_transform(best_pose)
            if counter % 20 == 0:
                rospy.loginfo("Calculating Probability")
                self.calc_prob()
                self.resample_points()

            counter += 1
            r.sleep()
            # self.delta_pose = (0,0,0)


if __name__ == '__main__':
    PF = ParticleFilter()
    PF.main()
    # PF.initial_resample_points(np.array([[1,1,1]]).T)
    # print(PF.particles)
    # print(np.std(PF.particles, axis=1))
