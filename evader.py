#!/usr/bin/env python3
import rclpy
import math as m

from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools
from unmanned_systems_ros2_pkg import Pathfinders
import numpy


def generate_random_waypoints(n_random_waypoints:int, max_val:int)->list:
    """generate random waypoints from 1 to 1"""
    
    random_wp_list = []
    for i in range(0,n_random_waypoints+1):
        rand_x = randint(0, max_val)
        rand_y = randint(0, max_val)
        random_wp_list.append((rand_x, rand_y))
        
    return random_wp_list

def compute_desired_heading(current_pos:list, des_pos:list) -> float:
    """compute desired heading based on positions"""
    return m.atan2(des_pos[1] - current_pos[1] , des_pos[0] - current_pos[0])

def compute_dist_error(current_pos:list, des_pos:list)->float:
    """compute distance error"""
    return m.dist(des_pos,current_pos)

def compute_heading_error(current_heading:float, des_heading:float) -> float:
    """compute heading error in radians"""
    return des_heading - current_heading

def gimme_da_loot(turtlebot:TurtleBotNode, waypoint:list) -> list:
    """helper function"""
    desired_heading = compute_desired_heading(
        turtlebot.current_position, waypoint)
    
    heading_error = compute_heading_error(
        turtlebot.orientation_euler[2], desired_heading)

    dist_error = compute_dist_error(
        turtlebot.current_position, waypoint)
    
    return [desired_heading, heading_error, dist_error]


def main() -> None:
    Start_x = 2
    Start_y = 1
    Goal_x = 7
    Goal_y = 2
    Obstacle_x = [5 , 5 , 5 , 5 , 5 , 0 , 1 , 2 , 3 , 3]
    Obstacle_y = [0 , 1 , 2 , 3 , 4 , 5 , 4 , 3 , 2 , 3]
    max_x = 15
    max_y = 15
    min_x = 0
    min_y = 0
    gs = 0.5
    domain_x = numpy.arange(min_x,max_x+gs,gs)
    domain_y = numpy.arange(min_y,max_y + gs,gs)
    domain_gs = numpy.arange(-gs,gs + gs,gs)    
    startime = Pathfinders.Djikstra(min_x,min_y,max_x,max_y,gs,domain_x,domain_y,domain_gs)
    x_path, y_path = startime.main(Goal_x,Goal_y,Start_x,Start_y,Obstacle_x,Obstacle_y,)
    rclpy.init(args=None)
    
    turtlebot_evader = TurtleBotNode.TurtleBotNode('turtle', 'evader')    
    turtlebot_evader.move_turtle(0.0,0.0)

    set_random = False
    is_done = False
    n_random_waypoints = 1
    heading_tol = 0.1; #radians
    dist_tolerance = 0.25 #meters
    
    turn_speed = 0.25 #rad/speed
    line_speed = 0.15 #m/s
    stop_speed = 0.0 
    waypoints = []
    i = 0
    if set_random == False:
        while i < len(x_path) - 1:
            waypoints.append([x_path[i],y_path[i]])
            i = i + 1
    else:
        waypoints = generate_random_waypoints(n_random_waypoints, 15)
    print(waypoints)
    while rclpy.ok():

        if is_done == True:
            print("I'm done")
            turtlebot_evader.move_turtle(stop_speed, stop_speed)
            rclpy.shutdown()
        waypoints = waypoints[::-1]
        for waypoint in waypoints:
            print("current waypoint is", waypoint)
            
            desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)

            while (abs(dist_error) >= dist_tolerance) or (abs(heading_error) >= heading_tol):

                # print("current heading is", m.degrees(turtlebot_evader.orientation_euler[2]))
                # print("desired heading is", m.degrees(desired_heading), heading_error)
        
                if abs(dist_error) >= dist_tolerance and  abs(heading_error) <= heading_tol:
                        turtlebot_evader.move_turtle(line_speed, stop_speed)
                elif abs(dist_error) < dist_tolerance and  abs(heading_error) >= heading_tol:
                    if heading_error > 0:
                        turtlebot_evader.move_turtle(stop_speed, turn_speed)
                    else:
                        turtlebot_evader.move_turtle(stop_speed, -turn_speed)
                else:
                    if heading_error > 0:
                        turtlebot_evader.move_turtle(line_speed, turn_speed)
                    else:
                        turtlebot_evader.move_turtle(line_speed, -turn_speed)
                
                desired_heading, heading_error, dist_error = gimme_da_loot(turtlebot_evader, waypoint)
                print(heading_error)
                rclpy.spin_once(turtlebot_evader)
                                
        #/we're done looping through our lists
        is_done = True
                        

if __name__=="__main__":
    main()
