#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2018 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    """
    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    print("home")
    arm.set_named_target("home")
    arm.go()
    
    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()
    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()
    """ 
    # アーム稼働
    gripper.set_joint_value_target([1.5, 1.5])
    gripper.go()
    # 手動で姿勢を指定するには以下のように指定
    # 0.624がアームの距離 
    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = -0.2
    target_pose.position.z = 0.09
    q = quaternion_from_euler( 1.57, 3.14, 3.14 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )	# 目標ポーズ設定
    arm.go()							# 実行
    # 二回目の移動
    arm.set_max_velocity_scaling_factor(0.1)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = -0.08
    target_pose.position.z = 0.08
    q = quaternion_from_euler( 1.57, 3.14, 3.14 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()                                                    # 実行
    # アーム稼働
    gripper.set_joint_value_target([0.57, 0.57])
    gripper.go()
    # 三回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = -0.08
    target_pose.position.z = 0.2
    q = quaternion_from_euler( 1.57, 3.14, 3.14 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()
    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()
    # アーム稼働
    gripper.set_joint_value_target([1.0, 1.5])
    gripper.go()
    # 準備姿勢
    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = 0.0
    target_pose.position.z = 0.0
    q = quaternion_from_euler( 0.0, -0.5, 0.0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()                            # 実行
    """
    # 投げる
    arm.set_max_velocity_scaling_factor(1.0)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.58 * math.sin(math.radians(30))
    target_pose.position.y = 0.0
    target_pose.position.z = 0.58 * math.cos(math.radians(30))
    q = quaternion_from_euler( 0.0, 1.5, 0.0 )  
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()                                                    # 実行
    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()
    # 移動後の手先ポーズを表示
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
