#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Quaternion, Vector3, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header
from nav_msgs.msg import Odometry
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import random
import tf2_ros
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs
import math
from scipy.stats import norm
from visualization_msgs.msg import Marker, MarkerArray
import pdb
import numpy as np


class ParticleFilter():

    def __init__(self):
        #
        rospy.init_node("ParticleFilter")
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)
        self.all_particles_pub = rospy.Publisher("/visualization_particles", MarkerArray, queue_size=10)
        self.init_particles_pub = rospy.Publisher("/visualization_init",MarkerArray,queue_size=10)
        self.initial_estimate_sub = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,self.pose_estimate_callback)

        # reference frame transform tools
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.occupancy_field = OccupancyField()
        self.particles = []
        self.number_of_particles = 20
        self.pos_std_dev = 0.25
        self.ori_std_dev = 25 * math.pi / 180
        self.lidar_std_dev = 0.05
        self.weights = []
        self.resample_threshold = 0.2
        self.scan = None
        self.prev_pose = None
        self.delta_pose = None
        self.initial_pose_estimate = None
        self.pose = None

        rospy.loginfo("Initialized")

    def initial_sample_points(self, initial_pose_estimate):
        # samples a uniform distribution of points
        for n in range(self.number_of_particles):
            x = (initial_pose_estimate[0] + random.normalvariate(0, self.pos_std_dev))
            y = (initial_pose_estimate[1] + random.normalvariate(0, self.pos_std_dev))
            t = (initial_pose_estimate[2] + random.normalvariate(0, self.ori_std_dev))
            self.particles.append((x,y,t))
            self.weights.append(1)


    def sample_points(self):
        # Takes the weights of each particle and resamples them according to that weight
        particles_to_resample = []
        particles_to_keep = []
        weights_to_keep = []
        for i in range(len(self.particles)):
            if(self.weights[i] < self.resample_threshold):
                particles_to_resample.append(i)
            else:
                particles_to_keep.append(i)
                weights_to_keep.append(self.weights[i])
        if(len(weights_to_keep) == 0):
            rospy.loginfo("No weights")
        for i in particles_to_resample:
            self.particles[i] = self.particles[random.choices(particles_to_keep,weights=weights_to_keep)[0]]


    def apply_odom_transform(self):
        # Takes rotation and translation from odom and transforms the particles accordingly. Also adds a bit of noise
        self.delta_pose = tuple([i - j for i, j in zip(self.pose, self.prev_pose)])

        for i, particle in enumerate(self.particles):

            if self.delta_pose[0] == 0 and self.delta_pose[1] == 0:
                rospy.logdebug('POSE DID NOT CHANGE')
            t = particle[2] + self.delta_pose[2]
            self.particles[i] = (particle[0] + self.delta_pose[0], particle[1] + self.delta_pose[1], t)

        # delta_x = math.cos(t * math.pi / 180) * self.delta_pose[0]
        # delta_y = math.sin(t * math.pi / 180) * self.delta_pose[1]

        self.prev_pose = self.pose

    def lidar_callback(self,msg):
        # Lidar Subscriber callback function.
        self.scan = msg.ranges;

    def odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        if self.prev_pose == None:
            self.prev_pose = (-x,-y,-t)
        else:
            self.pose = (-x,-y,-t)

        #self.plot_particles([self.pose], ColorRGBA(1, 1, 1, 0.5), self.init_particles_pub)

    def pose_estimate_callback(self,msg):
        rospy.logdebug("Callback Good")
        position = msg.pose.pose.position
        orientation = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        self.initial_pose_estimate = (position.x,position.y,orientation - math.pi)

    def calc_prob(self):
        # Reweight particles based on compatibility with laser scan

        for i, p in enumerate(self.particles):
            weight_sum = 0
            xs,ys = self.polar_to_cartesian(self.scan, np.radians(range(361)),p[2])
            lidar_points = [(x + p[0], y + p[1], 0) for x,y in zip(xs, ys)]

            # self.plot_particles(lidar_points, ColorRGBA(1, 1, 1, 0.5), self.init_particles_pub)
            # Average the probability associated with each LIDAR reading
            for x, y in zip(xs,ys):
                dist = self.occupancy_field.get_closest_obstacle_distance(p[0]+x,p[1]+y)
                prob = norm(0, self.lidar_std_dev).pdf(dist) / (0.4 / self.lidar_std_dev)
                weight_sum += prob
            self.weights[i] = weight_sum / 361

    def apply_particle_transform(self):
        # Take the LIDAR points and transform them into the global frame to be interpreted relative to map data

        pass

    def polar_to_cartesian(self, rs, thetas, theta_offset):
        # read the function name ok
        xs = []
        ys = []
        thetas = [t + theta_offset for t in thetas]

        for r, theta in zip(rs, thetas):
            xs.append(r * math.cos(theta))
            ys.append(r * math.sin(theta))

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
        r = rospy.Rate(1)
        while(not rospy.is_shutdown() and (self.scan == None or self.initial_pose_estimate == None or self.pose == None)):
            rospy.logwarn("Still Missing Something")
            r.sleep()


        # TODO: get inital pose estimate from RVIZ
        self.initial_sample_points(self.initial_pose_estimate)

        self.delta_pose = (4,4,1.5)

        while(not rospy.is_shutdown()):
            self.plot_particles(self.particles, ColorRGBA(0, 1, 0.5, 0.5), self.init_particles_pub)
            self.plot_particles(self.particles, ColorRGBA(1, 0, 0.5, 0.5), self.all_particles_pub)
            rospy.loginfo("before calc prob")
            self.calc_prob()
            rospy.loginfo("before sample points")
            self.sample_points()
            rospy.loginfo("before apply odom transform")
            self.apply_odom_transform()
            r.sleep()
            # self.delta_pose = (0,0,0)


if __name__ == '__main__':
    PF = ParticleFilter()
    PF.main()
