#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    quat = quaternion_from_euler(0,0,3.1416/2)
    initial_pose.pose.orientation.x = quat[0]
    initial_pose.pose.orientation.y = quat[1]
    initial_pose.pose.orientation.z = quat[2]
    initial_pose.pose.orientation.w = quat[3]
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 1.7
    quat = quaternion_from_euler(0,0,3.1416/2)
    goal_pose1.pose.orientation.x = quat[0]
    goal_pose1.pose.orientation.y = quat[1]
    goal_pose1.pose.orientation.z = quat[2]
    goal_pose1.pose.orientation.w = quat[3]

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.7
    goal_pose2.pose.position.y = 1.8
    quat = quaternion_from_euler(0,0,0)
    goal_pose2.pose.orientation.x = quat[0]
    goal_pose2.pose.orientation.y = quat[1]
    goal_pose2.pose.orientation.z = quat[2]
    goal_pose2.pose.orientation.w = quat[3]

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.9
    goal_pose3.pose.position.y = 0.9
    quat = quaternion_from_euler(0,0,-3.1416/2)
    goal_pose3.pose.orientation.x = quat[0]
    goal_pose3.pose.orientation.y = quat[1]
    goal_pose3.pose.orientation.z = quat[2]
    goal_pose3.pose.orientation.w = quat[3]

    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 0.9
    goal_pose4.pose.position.y = 0.1
    quat = quaternion_from_euler(0,0,-3.1416/2)
    goal_pose4.pose.orientation.x = quat[0]
    goal_pose4.pose.orientation.y = quat[1]
    goal_pose4.pose.orientation.z = quat[2]
    goal_pose4.pose.orientation.w = quat[3]

    goal_pose5 = PoseStamped()
    goal_pose5.header.frame_id = 'map'
    goal_pose5.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose5.pose.position.x = 1.7
    goal_pose5.pose.position.y = 0.0
    quat = quaternion_from_euler(0,0,0)
    goal_pose5.pose.orientation.x = quat[0]
    goal_pose5.pose.orientation.y = quat[1]
    goal_pose5.pose.orientation.z = quat[2]
    goal_pose5.pose.orientation.w = quat[3]

    goal_pose6 = PoseStamped()
    goal_pose6.header.frame_id = 'map'
    goal_pose6.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose6.pose.position.x = 1.8
    goal_pose6.pose.position.y = 1.95
    quat = quaternion_from_euler(0,0,3.1416/2)
    goal_pose6.pose.orientation.x = quat[0]
    goal_pose6.pose.orientation.y = quat[1]
    goal_pose6.pose.orientation.z = quat[2]
    goal_pose6.pose.orientation.w = quat[3]

    #navigator.goThroughPoses(goal_poses)
    navigator.goToPose(goal_pose1)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                / 1e9
                )
                + ' seconds.'
            )

    navigator.goToPose(goal_pose2)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                / 1e9
                )
                + ' seconds.'
            )


    navigator.goToPose(goal_pose3)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                / 1e9
                )
                + ' seconds.'
            )

    navigator.goToPose(goal_pose4)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                / 1e9
                )
                + ' seconds.'
            )

    navigator.goToPose(goal_pose5)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                / 1e9
                )
                + ' seconds.'
            )

    navigator.goToPose(goal_pose6)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                / 1e9
                )
                + ' seconds.'
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Foal failed!')
    else:
        print('goal has an invalid return status!')
    
    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()
