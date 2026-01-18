#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from pallet_gazebo.srv import verticesnnames
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R


def calculate_box_pose(vertices):
    vertices = np.array(vertices)
    center = np.mean(vertices, axis=0)

    x_axis = (vertices[1] - vertices[0]) / np.linalg.norm(vertices[1] - vertices[0])
    z_axis = (vertices[4] - vertices[0]) / np.linalg.norm(vertices[4] - vertices[0])
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()  # [x, y, z, w]

    pose = Pose()
    pose.position.x = center[0]
    pose.position.y = center[1]
    pose.position.z = center[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def spawn_box(model_name, pose):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        box_sdf = """
        <sdf version="1.6">
          <model name="box">
            <static>false</static>
            <link name="link">
              <inertial>
                <mass>0.8</mass>
              </inertial>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.5 0.5 0.5</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.5 0.5 0.5</size>
                  </box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """
        spawn_model(model_name=model_name, model_xml=box_sdf, initial_pose=pose, reference_frame="world")
        rospy.loginfo(f"Box {model_name} spawned successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn box {model_name}: {e}")


def delete_box(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(model_name=model_name)
        rospy.loginfo(f"Box {model_name} deleted successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to delete box {model_name}: {e}")


def request_vertices_and_names():
    rospy.loginfo("Waiting for service /collision_detect/vertices_names_gazebo")
    rospy.wait_for_service("/collision_detect/vertices_names_gazebo")
    try:
        vertices_client = rospy.ServiceProxy("/collision_detect/vertices_names_gazebo", verticesnnames)
        response = vertices_client()
        vertices = np.array(response.vertices).reshape(-1, 8, 3)
        names = response.names
        rospy.loginfo(f"Received {len(vertices)} sets of vertices and {len(names)} names.")
        return vertices, names
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call service: {e}")
        return None, None


if __name__ == "__main__":
    rospy.init_node('pallet_gazebo_node')

    # 서비스 요청
    vertices, names = None, None
    while True:
        vertices, names = request_vertices_and_names()
        if vertices is not None and names is not None:
            rospy.loginfo("Successfully received vertices and names.")
            break
        rospy.logwarn("Failed to receive vertices and names. Retrying...")
        rospy.sleep(1)  # 잠시 대기 후 재시도

    # 받은 데이터를 사용하여 박스 생성
    for i, vertices_set in enumerate(vertices):
        rospy.loginfo(f"Processing box {i+1}/{len(vertices)}")
        pose = calculate_box_pose(vertices_set)
        model_name = names[i] if i < len(names) else f"box{i+1}"
        rospy.loginfo(f"Spawning box: {model_name}")
        rospy.loginfo(f"x : {pose.position.x}, y : {pose.position.y}, z : {pose.position.z}\n"
                      f"x : {pose.orientation.x}, y : {pose.orientation.y}, z : {pose.orientation.z}, w : {pose.orientation.w}")
        spawn_box(model_name, pose)
        rospy.sleep(0.5)  # 각 모델 사이에 0.5초 대기

    rospy.loginfo("All boxes spawned. Keeping them for visualization.")
    rospy.sleep(15)  # 박스 유지 시간

    # 박스 삭제
    for i, name in enumerate(names):
        rospy.loginfo(f"Deleting box: {name}")
        delete_box(name)

    rospy.loginfo("All boxes deleted.")
    rospy.spin()  # 노드가 종료되지 않도록 유지
