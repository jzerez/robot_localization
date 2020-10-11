#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, LaserScan, Odometry, Quaternion, Vector3
from std_msgs.msg import ColorRBGA, Header
from nav_msgs.msg import Odometry
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import random
import tf2_ros
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs
import math
from scipy.stats import norm
from visualization_msgs.msg import Marker


class ParticleFilter():

    def __init__(self):
        #
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.odom_callback)

        # reference frame transform tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.occupancy_field = OccupancyField()
        self.particles = []
        self.number_of_particles = 1000
        self.pos_std_dev = 0.25
        self.ori_std_dev = 25 * pi / 180
        self.lidar_std_dev = 0.05
        self.weights = []
        self.resample_threshold = 0.2
        self.scan = None
        self.prev_pose = None
        self.delta_pose = None

    def initial_sample_points(self, initial_pose_estimate):
        # samples a uniform distribution of points
        for n in range(number_of_particles):
            x = (initial_pose_estimate[0] + random.normalvariate(0, self.pos_std_dev))
            y = (initial_pose_estimate[1] + random.normalvariate(0, self.pos_std_dev))
            t = (initial_pose_estimate[2] + random.normalvariate(0, self.ori_std_dev))
            self.particles[n] = (x,y,t)
            self.weights[n] = 1


    def sample_points(self, weights):
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

        for i in particles_to_resample:
            self.particles[i] = self.particles[random.choices(particles_to_keep,weights=weights_to_keep)]


    def apply_odom_transform(self, particle):
        # Takes rotation and translation from odom and transforms the particles accordingly. Also adds a bit of noise

        t = particle[2] + self.delta_pose[2]
        # delta_x = math.cos(t * math.pi / 180) * self.delta_pose[0]
        # delta_y = math.sin(t * math.pi / 180) * self.delta_pose[1]

        return (particle[0] + self.delta_pose[0], particle[1] + self.delta_pose[1], t)



    def lidar_callback(self,msg):
        # Lidar Subscriber callback function.
        self.scan = msg.ranges;

    def odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = euler_from_quaternion([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z])[0]
        pose = (x,y,t)
        self.delta_pose = tuple([i - j for i, j in zip(pose, self.prev_pose)])
        self.prev_pose = pose


    def calc_prob(self):
        # Reweight particles based on compatibility with laser scan

        for i, p in enumerate(self.particles):
            weight_sum = 0
            xs,ys = polar_to_cartesian(msg.ranges,math.radians(range(361)),p[2])
            # Average the probability associated with each LIDAR reading
            for x, y in zip(xs,yz):
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

    def main(self):

        pass

if __name__ == '__main__':
    PF = ParticleFilter()
    PF.main()
