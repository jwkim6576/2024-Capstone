# ROS 관련 라이브러리
import rospy
from std_msgs.msg import Float32, String, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from palletizing_system.srv import servoing, servoingResponse
from dsr_msgs.srv import (
    MoveLine, MoveLineRequest,
    MoveJoint, MoveJointRequest,
    GetCurrentPosx, GetCurrentRotm
)

# 수학 및 과학 계산 관련 모듈
from math import cos, sin, pi, atan2, acos, sqrt, radians
import numpy as np
from scipy.spatial.transform import Rotation as R

# 시각화 및 플롯팅 라이브러리
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 컴퓨터 비전 및 3D 처리 라이브러리
import cv2 as cv
import open3d as o3d

# 기타 유틸리티 및 데이터 처리
import time
import struct
import random
from io import BytesIO

# 경고 및 오류 관리
import warnings
from sklearn.exceptions import ConvergenceWarning

# 최적화 설정
np.set_printoptions(precision=4, suppress=True)  # 소수점 4자리 출력 설정
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=ConvergenceWarning)

def pointcloud_callback(msg: PointCloud2):
    global realsense_pointcloud
    realsense_pointcloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

def create_axis_lines(scale=5.0):
    """Create axis lines for visualization."""
    axis_lines = o3d.geometry.LineSet()
    
    # Scale the length of the axes to 50 times
    scaled_length = scale * 50  # 50배 확대
    points = np.array([[0, 0, 0], [scaled_length, 0, 0], 
                       [0, 0, 0], [0, scaled_length, 0], 
                       [0, 0, 0], [0, 0, scaled_length]])
    lines = np.array([[0, 1], [2, 3], [4, 5]])
    
    axis_lines.points = o3d.utility.Vector3dVector(points)
    axis_lines.lines = o3d.utility.Vector2iVector(lines)
    
    # Set colors for each axis (X: Red, Y: Green, Z: Blue)
    colors = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    axis_lines.colors = o3d.utility.Vector3dVector(colors)
    
    return axis_lines

def filter_point_cloud(pcd):
    """
    Filters the points in a point cloud to include:
    - x > 0
    - z > 0
    - y >= -200
    """
    points = np.asarray(pcd.points)
    mask = (points[:, 0]>= -2) & (points[:, 1] > 0) & (points[:, 2] > 0)  # Apply the conditions
    filtered_pcd = pcd.select_by_index(np.where(mask)[0])
    return filtered_pcd

def visualize_point_clouds(pcd1, pcd2, axis_lines):
    # Create a visualizer objecㅅ
    pcd1 = filter_point_cloud(pcd1)
    pcd2 = filter_point_cloud(pcd2)
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add the point clouds to the visualizer
    vis.add_geometry(pcd1)
    vis.add_geometry(pcd2)
    vis.add_geometry(axis_lines)

    # Optionally, set different colors for each point cloud for better distinction
    pcd1.paint_uniform_color([1, 0, 0])  # Red color
    pcd2.paint_uniform_color([0, 1, 0])  # Green color

    # Optionally, set a view control for better visualization
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)

    # Run the visualizer
    vis.run()

    # Combine the point clouds
    combined_pcd = pcd1 + pcd2
    
    # Close the visualizer
    vis.destroy_window()

def pointcloud_to_robot_coordinate(cloud_from_camera):
    get_current_pose_srv = rospy.ServiceProxy('/dsr01m1013/aux_control/get_current_posx', GetCurrentPosx)
    get_current_rotm_srv = rospy.ServiceProxy('/dsr01m1013/aux_control/get_current_rotm', GetCurrentRotm)
    
    get_current_pose_response = get_current_pose_srv(0)
    get_current_rotm_response = get_current_rotm_srv(0)


    camera2endeffector = np.array([ [0.9996215749858904, -0.01240737295999214, -0.02455125086343612, -25.14063285848638],
                                    [0.01112504270560043, 0.9986009416289027, -0.05169519129152499, -105.0359579400699],
                                    [0.02515830374898681, 0.05140249482369946, 0.9983610786075213, -12.21662738431984],
                                    [0, 0, 0, 1]])


    endeffector2base = np.array([[get_current_rotm_response.rot_matrix[0].data[0], get_current_rotm_response.rot_matrix[0].data[1], get_current_rotm_response.rot_matrix[0].data[2], get_current_pose_response.task_pos_info[0].data[0]],
                                 [get_current_rotm_response.rot_matrix[1].data[0], get_current_rotm_response.rot_matrix[1].data[1], get_current_rotm_response.rot_matrix[1].data[2], get_current_pose_response.task_pos_info[0].data[1]],
                                 [get_current_rotm_response.rot_matrix[2].data[0], get_current_rotm_response.rot_matrix[2].data[1], get_current_rotm_response.rot_matrix[2].data[2], get_current_pose_response.task_pos_info[0].data[2]],
                                 [0, 0, 0, 1]])

    camera2base = np.dot(endeffector2base, camera2endeffector)
    # Extract the rotation matrix and translation vector from camera2base
    rotation_camera2base = camera2base[:3, :3]
    translation_camera2base = camera2base[:3, 3] * 0.001            # 맞는 것 같은데..?
    # print(f"Rotm : {rotation_camera2base}")
    # print(f"Translation : {translation_camera2base}")

    # Convert Open3D PointCloud to numpy array of points
    pc_np = np.asarray(cloud_from_camera.points)
    
    # Transform the point cloud
    transformed_points = np.dot(rotation_camera2base, pc_np.T).T + translation_camera2base

    # Create a new Open3D PointCloud for the transformed points
    robot_coordinate_cloud = o3d.geometry.PointCloud()
    robot_coordinate_cloud.points = o3d.utility.Vector3dVector(transformed_points)
    
    return robot_coordinate_cloud

def scale_point_cloud(pcd, scale_factor):
    # Create a scaling matrix
    scaling_matrix = np.eye(4)
    scaling_matrix[0, 0] = scale_factor  # Scale x-axis
    scaling_matrix[1, 1] = scale_factor  # Scale y-axis
    scaling_matrix[2, 2] = scale_factor  # Scale z-axis
    
    # Apply scaling transformation
    pcd.transform(scaling_matrix)
    return pcd

def extract_vertices_from_synthetic_points(synthetic_points):
    # 포인트 클라우드의 점들을 numpy 배열로 변환
    points = np.asarray(synthetic_points.points)

    # 중복된 점들을 제거
    unique_points = np.unique(points, axis=0)

    # 중점 계산
    centroid = np.mean(unique_points, axis=0)

    # 각 점과 중점 간의 거리 계산
    distances = np.linalg.norm(unique_points - centroid, axis=1)

    # 거리가 가장 먼 8개의 점의 인덱스 찾기
    farthest_indices = np.argsort(distances)[-8:]

    # 해당 점들을 정점으로 반환
    vertices = unique_points[farthest_indices] * 100

    return vertices

def sample_points_on_cuboid(vertices, num_points_per_face=100):
    # Define faces using vertex indices
    faces = [
        (0, 1, 2, 3),  # Bottom face
        (4, 5, 6, 7),  # Top face
        (0, 1, 5, 4),  # Side face 1
        (2, 3, 7, 6),  # Side face 2
        (1, 2, 6, 5),  # Front face
        (0, 3, 7, 4)   # Back face
    ]

    points = []

    for face in faces:
        v0, v1, v2, v3 = [vertices[i] for i in face]
        # Sample points in the face's plane
        for u in np.linspace(0, 1, int(np.sqrt(num_points_per_face))):
            for v in np.linspace(0, 1, int(np.sqrt(num_points_per_face))):
                point = (1-u)*(1-v)*v0 + u*(1-v)*v1 + u*v*v2 + (1-u)*v*v3
                points.append(point)

    # Convert list of points to numpy array
    points = np.array(points)

    # Create Open3D PointCloud object and assign points
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud

def extract_vertices_from_synthetic_points(synthetic_points):
    # 포인트 클라우드의 점들을 numpy 배열로 변환
    points = np.asarray(synthetic_points.points)

    # 중복된 점들을 제거
    unique_points = np.unique(points, axis=0)

    # 중점 계산
    centroid = np.mean(unique_points, axis=0)

    # 각 점과 중점 간의 거리 계산
    distances = np.linalg.norm(unique_points - centroid, axis=1)

    # 거리가 가장 먼 8개의 점의 인덱스 찾기
    farthest_indices = np.argsort(distances)[-8:]

    # 해당 점들을 정점으로 반환
    vertices = unique_points[farthest_indices] * 100

    return vertices

def correct_rough_vertices(rough_vertices):
    global fitness
    # print(f"input vertices : \n{rough_vertices}")
    pc_np = np.array(list(realsense_pointcloud))
    cloud_from_camera = o3d.geometry.PointCloud()
    cloud_from_camera.points = o3d.utility.Vector3dVector(pc_np)
    robot_coordinate_cloud = pointcloud_to_robot_coordinate(cloud_from_camera)
    rough_pointcloud = sample_points_on_cuboid(rough_vertices, num_points_per_face=100)
    rough_pointcloud = scale_point_cloud(rough_pointcloud, 0.01)
    robot_coordinate_cloud = scale_point_cloud(robot_coordinate_cloud, 10)
    axis_lines = create_axis_lines(scale=0.1)
    # visualize_point_clouds(robot_coordinate_cloud, rough_pointcloud, axis_lines)            # ICP 전 상황
    
    # start_time = time.time()
    robot_coordinate_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    rough_pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # end_time = time.time()
    # execution_time = end_time - start_time
    # print(f"execution_time : {execution_time}")

    threshold = 0.1
    trans_init = np.eye(4)  # Initial transformation matrix
    icp_convergence_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-8, relative_rmse=1e-8, max_iteration=100)
    icp_result = o3d.pipelines.registration.registration_icp(
        rough_pointcloud, robot_coordinate_cloud, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(), icp_convergence_criteria)
    transformation = icp_result.transformation
    fitness = icp_result.fitness
    print(f" fitness : {icp_result.fitness}")
    rough_pointcloud.transform(transformation)
    a = extract_vertices_from_synthetic_points(rough_pointcloud)
    # print(f"from pointcloud : {a}")
    # visualize_point_clouds(robot_coordinate_cloud, rough_pointcloud, axis_lines)        # ICP 후 상황
    
    scale = 1000

    scaled_transformation = transformation.copy()
    # translation 부분을 배율만큼 곱함
    scaled_transformation[0:3, 3] *= scale
    Scaled_input = rough_vertices * 10
    vertices_homogeneous = np.hstack((Scaled_input, np.ones((Scaled_input.shape[0], 1))))  # 동차 좌표로 변환
    transformed_vertices = scaled_transformation @ vertices_homogeneous.T  # 변환 행렬과 곱함
    transformed_vertices = transformed_vertices.T[:, :3]  # 변환된 결과에서 x, y, z 좌표만 추출
    transformed_vertices = transformed_vertices * 0.1
    # print(f"transformation : \n{scaled_transformation}")
    # print(f"transformed_vertices : {transformed_vertices}")
    


    correct_vertices = transformed_vertices
    return correct_vertices

def handle_detection_service(request):
    # 요청에서 rough_vertices 가져오기
    rough_vertices = np.array(request.rough_vertices)  # NumPy 배열로 변환
    # print(rough_vertices.size)
    # 24개의 원소가 있는지 확인 후 (8, 3) 형태로 변환
    if rough_vertices.size != 24:
        rospy.logerr("Invalid number of vertices. Expected 24 values (8 points with 3 coordinates each).")
        return None

    # (8, 3) 형태로 변환하여 좌표 출력
    rough_vertices = rough_vertices.reshape((8, 3))
    # print(f"rough_vertices : \n{rough_vertices}")


    correct_vertices = correct_rough_vertices(rough_vertices)

    # print(f"correct_vertices : \n{correct_vertices}")

    # 응답 생성 및 필드 채우기
    response = servoingResponse()
    response.correct_vertices = correct_vertices.flatten()  # 응답에 1차원 배열로 설정
    response.fitness = fitness

    return response

def main():
    rospy.init_node('servoing_node')
    pointcloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback)

    try:
        service = rospy.Service('/collision_detect/servoing', servoing, handle_detection_service)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    rospy.spin()

if __name__ == '__main__':
    main()
