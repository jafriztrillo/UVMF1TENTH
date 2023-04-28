#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    #GLOBAL VARIABLES
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs

        self.speed = 0.0
        # TODO: Publish to drive
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 1000)

        # TODO: Subscribe to LIDAR
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10
        )

    def preprocess_lidar(self, lidar_msg):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        #  Determine "n" average range and the distance treshhold "t". Want a mean over a given window
        proc_ranges = np.convolve(lidar_msg, np.ones(5), 'same') / 5
        # Reject all scans >3m
        proc_ranges = np.clip(proc_ranges, 0.0, 3.0) # 3.0 meters
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        startIdx = 0
        endIdx = 0
        currGap = 0
        longestGap = 0
        for idx in range(len(free_space_ranges)):
            if(free_space_ranges[idx] <= 0.0):
                startIdx = idx
            elif(free_space_ranges[idx] > 0.0):
                currGap = idx - startIdx
                if(currGap > longestGap):
                    longestGap = currGap
                    startGap = startIdx
                    endIdx = idx

        return (startGap, endIdx)

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
       """       
        best_gap_idx = np.convolve(ranges[start_i:end_i], np.ones(90), 'same') / 90
        return best_gap_idx.argmax() + start_i
    
    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def lidar_callback(self, lidar_msg):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = lidar_msg.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        #Find closest point to LiDAR
        """
        Find the minimum point in this array before setting a bubble around it
        """
        #Eliminate all points inside 'bubble' (set them to zero) 
        """
        Find the minimum range and the maximum range of the closest_point and write all range data within this area equal to zero
        """
        closest_point_idx = proc_ranges.argmin()
        
        bubble_min_idx = closest_point_idx - 180
        if bubble_min_idx < 0: bubble_min_idx = 0
        bubble_max_idx = closest_point_idx + 180
        if bubble_max_idx > len(proc_ranges): bubble_max_idx = len(proc_ranges)-1
        proc_ranges[bubble_min_idx:bubble_max_idx] = 0

        #Find max length gap 
        (gapMin, gapMax) = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        bestGap = self.find_best_point(gapMin,gapMax,proc_ranges)

        #Process the bestGap and find the steering angle
        steer_angle = lidar_msg.angle_min + bestGap*lidar_msg.angle_increment

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        
        if(abs(steer_angle * (180/np.pi)) <= 10):
            drive_msg.drive.speed = 1.0
        elif(abs(steer_angle * (180/np.pi)) <= 20):
            drive_msg.drive.speed = 0.7
        elif(abs(steer_angle * (180/np.pi)) <= 40):
            drive_msg.drive.speed = 0.5
        else:
            drive_msg.drive.speed = 0.4
        # drive_msg.drive.speed = 5.0
        drive_msg.drive.steering_angle = steer_angle
        print("Steering angle is {}".format(steer_angle * (180 / np.pi)))
        self.publisher_.publish(drive_msg) # Publish drive message


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
