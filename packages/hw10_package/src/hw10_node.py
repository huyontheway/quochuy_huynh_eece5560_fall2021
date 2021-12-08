#!/usr/bin/env python3

import rospy
# Brings in the SimpleActionClient
import actionlib
from example_service.srv import Fibonacci, FibonacciResponse
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import example_action_server.msg


def calc_fibonacci_client(order):
    rospy.loginfo("Sevice started")
    start_srv = rospy.get_time()
    rospy.loginfo(start_srv)
    
    #Wait for the service to become available
    rospy.wait_for_service('calc_fibonacci')
    try:
        #Set up the service
        calc_fibonacci = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        #Request the service
        result = calc_fibonacci(order)
        
        rospy.loginfo("Service finished")
        end_srv = rospy.get_time()
        rospy.loginfo(end_srv)
        rospy.loginfo("Service total time")
        rospy.loginfo(end_srv - start_srv)
        
        return result.sequence  # Return the result
    except rospy.ServiceException as e: 
        print("Service call failed: %s" %e)

def fibonacci_client(ord):
    try:
        
        rospy.loginfo("Action client started")
        start_act = rospy.get_time()
        rospy.loginfo(start_act)
        
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        
        # Time stamp at sending goal
        rospy.loginfo("Sending goal")
        start_goal = rospy.get_time()
        rospy.loginfo(start_goal)
        
        # Creates a goal to send to the action server.
        goal = example_action_server.msg.FibonacciGoal(order=ord)
    
        # Sends the goal to the action server.
        client.send_goal(goal)
    
        # Time stamp at receiving goal
        rospy.loginfo("Received goal")
        end_goal = rospy.get_time()
        rospy.loginfo(end_goal)
        
        rospy.loginfo("total time sending and receving goal")
        rospy.loginfo(end_goal - start_goal)
        
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        
        # time stamp at which action client finished
        rospy.loginfo("Action client finished")
        end_act = rospy.get_time()
        rospy.loginfo(end_act)
        
        rospy.loginfo("Total time for action client")
        rospy.loginfo(end_act - start_act)
    
        # Prints out the result of executing the action
        return client.get_result()  # A FibonacciResult
    except rospy.ServiceException as e: 
        print("Service call failed: %s" %e)
    
if __name__ == "__main__":
    rospy.init_node('hw10_node',anonymous=True)
    
    # Request Fibonacci sequence from service
    res_srv = calc_fibonacci_client(14)
    rospy.loginfo(res_srv)
    
    # request Fibonacci sequence from action
    res_act = fibonacci_client(14)
    rospy.loginfo(res_act)
    
