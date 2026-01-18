# ROS 관련 라이브러리
import rospy
from dsr_msgs.srv import (
    MoveLine, MoveLineRequest,
    MoveJoint, MoveJointRequest,
    GetCurrentPosx, GetCurrentRotm,
    MoveJointx, MoveJointxRequest,
    GetCurrentPose, GetCurrentPoseRequest,
    GetCurrentPosj
)
from palletizing_system.srv import (
    multi_viewing_pose, multi_viewing_poseRequest,
    vision_vertices, vision_verticesResponse
)
from vision_info_srv.srv import vision_info_srv, vision_info_srvResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import (
    Image, CameraInfo, PointCloud2, PointField
)
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, String, ColorRGBA
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Quaternion, PoseArray, Pose, Vector3

# 수학 및 과학 계산 관련 라이브러리
import numpy as np
from math import cos, sin, pi, atan2, acos, sqrt, radians
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
from scipy.stats import zscore
import scipy.signal as signal

# 경고 및 오류 관리
import warnings
from sklearn.exceptions import ConvergenceWarning
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=ConvergenceWarning)

# 컴퓨터 비전 및 3D 처리 라이브러리
import cv2 as cv
import open3d as o3d
from ultralytics import YOLO

# 머신러닝 및 클러스터링 관련 라이브러리
from sklearn.cluster import (
    KMeans, AgglomerativeClustering, DBSCAN
)
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics import silhouette_score

# 시각화 및 플롯팅 라이브러리
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
matplotlib.use('TkAgg')

# 기타 유틸리티
import random
from io import BytesIO
import struct

Image_publisher = rospy.Publisher('yolo_image', Image, queue_size=10)

model = YOLO('~/catkin_ws/src/weight/best.pt') # 원하는 절대위치 또는 상대위치를 삽입
print("YOLO is on!")
bridge = CvBridge()
depth_image = None
color_camera_matrix = np.zeros((3, 3), dtype=np.float32)
depth_camera_matrix = np.zeros((3, 3), dtype=np.float32)
frame_count = 0

color_distortion_coefficient = np.zeros((5, 1), dtype=np.float32)
depth_distortion_coefficient = np.zeros((5, 1), dtype=np.float32)
name_list = ['color_logo', 'black_logo', 'round_logo', 'blue_robot', 'red_robot']
label_dict = {
    'color_logo': np.array([
        [0, 0, 0],
        [-2.8, -2.8, 0],
        [5.6, -2.8, 0],
        [2.8, 0, 0]
    ], dtype=np.float32),

    'black_logo': np.array([
        [0, 0, 0],
        [-3, -3, 0],
        [6.0, -3, 0],
        [3, 0, 0]
    ], dtype=np.float32),

    'round_logo': np.array([
        [0.00, 0.00, 0],
        [-2.13, 1.60, 0],
        [2.99, 1.63, 0],
        [0.88, 0.00, 0]
    ], dtype=np.float32),

    'blue_robot': np.array([
        [0.00, 0.00, 0],
        [1.60, 6.01, 0],   # Adjusted from [3.19, 12.02, 0]
        [4.08, 6.14, 0],   # Adjusted from [8.15, 12.27, 0]
        [6.29, -1.42, 0]   # Adjusted from [12.59, -2.83, 0]
    ], dtype=np.float32),

    'red_robot': np.array([
        [0.00, 0.00, 0],
        [-2.36, 2.70, 0],   # Adjusted from [-4.72, 5.40, 0]
        [0.50, 5.17, 0],    # Adjusted from [0.99, 10.33, 0]
        [4.04, 2.48, 0]     # Adjusted from [8.08, 4.97, 0]
    ], dtype=np.float32)
}


# DEG to RAD 변환 함수
def DEG2RAD(deg):
    return deg * np.pi / 180.0

# 3x3 행렬과 3x1 벡터 곱셈
def matVecMult(mat, vec):
    return np.dot(mat, vec)

# Z축 회전 행렬
def rotZ(angle):
    rad = DEG2RAD(angle)
    return np.array([
        [np.cos(rad), -np.sin(rad), 0.0],
        [np.sin(rad),  np.cos(rad), 0.0],
        [0.0, 0.0, 1.0]
    ])

# Y축 회전 행렬
def rotY(angle):
    rad = DEG2RAD(angle)
    return np.array([
        [np.cos(rad), 0.0, np.sin(rad)],
        [0.0, 1.0, 0.0],
        [-np.sin(rad), 0.0, np.cos(rad)]
    ])

# ZYZ 회전 행렬을 얻는 함수
def getZYZRotationMatrix(yawZ1, pitchY, rollZ2):
    Rz1 = rotZ(yawZ1)
    Ry = rotY(pitchY)
    Rz2 = rotZ(rollZ2)

    # Rz1 * Ry * Rz2 행렬 곱
    return np.dot(np.dot(Rz1, Ry), Rz2)

# RPY 값을 받아 회전 벡터 반환
def rpy2vector(yawZ1, pitchY, rollZ2):
    rotationMatrix = getZYZRotationMatrix(yawZ1, pitchY, rollZ2)
    unitVectors = np.identity(3)  # 단위 벡터 [1, 0, 0], [0, 1, 0], [0, 0, 1]

    resultVec = np.dot(rotationMatrix, unitVectors.T).T  # 각 벡터에 회전 적용
    return resultVec

# 평행이동된 posx를 받아 평행이동 전 posx(moveline의 명령) 구함
def posx_calculate(posx, object_x, object_y, object_z):
    temp_posx = posx.copy()
    unitVec = rpy2vector(posx[3],posx[4],posx[5])

    forward = np.array([object_x, object_y, object_z])

    for i in range(3):
        for j in range(3):
            temp_posx[j] = temp_posx[j] +unitVec[i][j] * forward[i]

    return temp_posx

def get_current_pose(space_type): # 0 : joint 1: space
    # ROS 노드가 초기화되어 있는지 확인
    # if not rospy.core.is_initialized():
    #     rospy.init_node('get_current_pose_client', anonymous=True)
    
    # 서비스 클라이언트 생성
    rospy.wait_for_service('/dsr01m1013/system/get_current_pose')
    try:

        get_current_pose = rospy.ServiceProxy('/dsr01m1013/system/get_current_pose', GetCurrentPose)
        req = GetCurrentPoseRequest()
        req.space_type = space_type

        response = get_current_pose(req)

        current_pose = response.pos

        return current_pose  # 위치 값 리스트를 반환

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def movejx(posx, vel, acc, time, object_x=0, object_y=0, object_z=0, sol=2):


    # 서비스 클라이언트 생성
    rospy.wait_for_service('/dsr01m1013/motion/move_jointx')
    try:
        move_jointx = rospy.ServiceProxy('/dsr01m1013/motion/move_jointx', MoveJointx)
        prepose = np.array(get_current_pose(0))    


        # posxG 계산
        posxg = posx_calculate(posx, object_x, object_y, object_z - 230)

        # 서비스 요청 데이터 설정
        req = MoveJointxRequest()
        req.pos = posxg
        req.vel = vel
        req.acc = acc
        req.sol = sol
        req.time = time
        req.radius = 0
        req.mode = 0
        req.blendType = 0
        req.syncType = 0

        response = move_jointx(req)

        if response:
            current_pose = np.array(get_current_pose(0))
            posdiff = current_pose - prepose
            if(np.linalg.norm(posdiff) < 0.1):
                return False
            else :
                return True
        else:
            rospy.logerr("Failed to call service move_joint")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def is_rotation_matrix(Rotm):
    """
    Check if a matrix is a valid rotation matrix.
    """
    # Check if Rotm is orthogonal
    identity_matrix = np.eye(3)
    orthogonality = np.allclose(np.dot(Rotm.T, Rotm), identity_matrix, atol=1e-6)
    # print(f"orthogonality : {orthogonality}")
    
    # Check if determinant is 1
    determinant = np.linalg.det(Rotm)
    determinant_check = np.isclose(determinant, 1, atol=1e-6)
    # print(f"determinant_check : {determinant_check}")
    return orthogonality and determinant_check

def movej(pos, vel, acc, time):
    # ROS 노드 초기화

    # 서비스 클라이언트 생성
    rospy.wait_for_service('/dsr01m1013/motion/move_joint')
    try:
        move_joint_service = rospy.ServiceProxy('/dsr01m1013/motion/move_joint', MoveJoint)

        # 서비스 요청 생성
        srv = MoveJointRequest()
        srv.pos = pos
        srv.vel = vel
        srv.acc = acc
        srv.time = time
        srv.radius = 0
        srv.mode = 0
        srv.blendType = 0
        srv.syncType = 0

        # 서비스 호출
        response = move_joint_service(srv)
        # if response:
        #     rospy.loginfo("Success")
        # else:
        #     rospy.logerr("Failed to call service move_joint")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def moveline(posx, vel, acc, time_):
    rospy.wait_for_service('/dsr01m1013/motion/move_line')
    try:
        move_line = rospy.ServiceProxy('/dsr01m1013/motion/move_line', MoveLine)
        req = MoveLineRequest()
        req.pos = posx
        req.vel = vel
        req.acc = acc
        req.time = time_
        req.radius = 0
        req.mode = 0
        req.blendType = 0
        req.syncType = 0

        response = move_line(req)
        
        #rospy.loginfo("MoveLine Success: %f %f %f %f %f %f", *posx)
    except rospy.ServiceException as e:
        #rospy.logerr("Failed to call service move_line: %s", e)
        rospy.signal_shutdown("Service call failed")

def get_rpy(R):
    if R[2, 2] == 1.0:
        beta = np.pi
        alpha = 0
        gamma = np.arctan2(R[0, 1], R[0, 0])
    elif R[2, 2] == -1.0:
        beta = np.pi
        alpha = 0
        gamma = np.arctan2(-R[0, 1], R[0, 0])
    else:
        beta = np.arccos(R[2, 2])
        sin_beta = np.sin(beta)
        alpha = np.arctan2(R[1, 2] / sin_beta, R[0, 2] / sin_beta)
        gamma = np.arctan2(R[2, 1] / sin_beta, -R[2, 0] / sin_beta)
    return np.degrees(alpha), np.degrees(beta), np.degrees(gamma)

def color_camera_info_callback(msg):
    global color_camera_matrix, color_distortion_coefficient, camera_matrix, dist_coeffs
    color_camera_matrix = np.array(msg.K).reshape(3, 3)
    color_distortion_coefficient = np.array(msg.D).reshape(-1, 1)
    dist_coeffs = color_distortion_coefficient
    camera_matrix = color_camera_matrix

def depth_msg_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.logerr(e)

def pointcloud_callback(msg: PointCloud2):
    global realsense_pointcloud
    realsense_pointcloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

def random_points_inside_quadrilateral(points, num_points):
    random_points = []
    
    if len(points) == 4:
        # Divide the quadrilateral into two triangles
        triangle1 = np.array([points[0], points[1], points[2]])
        triangle2 = np.array([points[2], points[3], points[0]])
        
        for _ in range(num_points):
            # Randomly choose one of the triangles to sample a point from
            triangle = triangle1 if random.random() < 0.5 else triangle2
            
            # Generate random barycentric coordinates
            r1, r2 = random.random(), random.random()
            if r1 + r2 > 1:
                r1, r2 = 1 - r1, 1 - r2
            
            # Calculate the random point within the triangle
            random_point = (1 - r1 - r2) * triangle[0] + r1 * triangle[1] + r2 * triangle[2]
            random_points.append(random_point)
    
    return random_points

def print_objects_numbers(objects): 
    for label, data in objects.items():
        if len(data['label']) != 0 :
            print(f"Label: {label}")
            print(f"  Number of entries: {len(data['label'])}")
            print("")

def get_coordinate_depth(depth_image, pixel_coordinate):
    height, width = depth_image.shape[:2]

    fx = color_camera_matrix.flatten()[0]
    fy = color_camera_matrix.flatten()[4]
    cx = color_camera_matrix.flatten()[2]
    cy = color_camera_matrix.flatten()[5]
    u = pixel_coordinate[0]
    v = pixel_coordinate[1] 

    if u < 0 or u >= width or v < 0 or v >= height:
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)

    depth = depth_image[v, u]
    if depth != 0:
        x = (u - cx) / fx
        y = (v - cy) / fy

        point = [
            float(depth * x / 10.0),
            float(depth * y / 10.0),
            float(depth / 10.0)
        ]
        return np.array(point, dtype=np.float32)
    else:
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)

def align_z_axis_and_get_new_rotation_matrix(plane_z_axis, pnp_rotation_matrix):
    """
    평면화된 z축과 pnp 회전행렬을 입력받아
    새로운 회전행렬을 반환합니다.
    
    Parameters:
        plane_z_axis (np.ndarray): 평면화된 z축 벡터 (길이 1의 벡터)
        pnp_rotation_matrix (np.ndarray): pnp로 얻은 회전행렬 (3x3 행렬)
        
    Returns:
        np.ndarray: 평면화된 z축에 맞게 조정된 새로운 회전행렬 (3x3 행렬)
    """
    
    # 1. 평면화된 z축 벡터를 정규화
    plane_z_axis = plane_z_axis / np.linalg.norm(plane_z_axis)
    
    # 2. pnp 회전행렬의 z축 벡터를 추출
    pnp_z_axis = pnp_rotation_matrix[:, 2]
    pnp_z_axis = pnp_z_axis / np.linalg.norm(pnp_z_axis)  # 정규화
    
    # 3. pnp의 z축을 평면화된 z축으로 맞추기 위한 회전축과 각도 계산
    axis = np.cross(pnp_z_axis, plane_z_axis)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.clip(np.dot(pnp_z_axis, plane_z_axis), -1.0, 1.0))
    
    # 4. 회전행렬 생성 (Rodrigues' rotation formula)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    
    # 5. pnp 회전행렬의 z축을 평면화된 z축으로 맞추기 위해 새로운 회전행렬을 계산
    new_rotation_matrix = R @ pnp_rotation_matrix
    
    return new_rotation_matrix

def get_cuboid_vertices_from_face_center(face_center, rvec, width, height, depth):
    # Define the half dimensions
    w = 10 * width / 2.0
    h = 10 * height / 2.0
    d = 10 * depth / 2.0

    # Define the 8 vertices of the cuboid in the local coordinate system
    local_vertices = np.array([
        [-w, -h, -d],
        [w, -h, -d],
        [w, h, -d],
        [-w, h, -d],
        [-w, -h, d],
        [w, -h, d],
        [w, h, d],
        [-w, h, d]
    ], dtype=np.float32)

    # Calculate the cuboid center by offsetting the face center
    # Here we assume the face is the front face, so we offset by depth / 2
    cuboid_center_offset = np.array([0, 0, -d])
    
    # Convert rvec to rotation matrix
    R, _ = cv.Rodrigues(rvec)
    
    # if R[2, 2] < 0:
    #     # z축을 반대로 뒤집음
    #     R[:, 2] = -R[:, 2]
        
        
    # print(R)

    # Rotate the offset
    rotated_offset = np.dot(cuboid_center_offset, R.T)
    
    # Calculate the cuboid center in world coordinates
    cuboid_center = face_center + rotated_offset

    # Apply the rotation to the local vertices
    rotated_vertices = np.dot(local_vertices, R.T)

    # Translate the vertices to the cuboid center point
    vertices = rotated_vertices + cuboid_center

    return vertices

def get_z_reversed_cuboid_vertices_from_face_center(face_center, rvec, width, height, depth):
    # Define the half dimensions
    w = width / 2.0 * 10
    h = height / 2.0 * 10
    d = depth / 2.0 * 10

    # Define the 8 vertices of the cuboid in the local coordinate system
    local_vertices = np.array([
        [-w, -h, -d],
        [w, -h, -d],
        [w, h, -d],
        [-w, h, -d],
        [-w, -h, d],
        [w, -h, d],
        [w, h, d],
        [-w, h, d]
    ], dtype=np.float32)

    # Calculate the cuboid center by offsetting the face center
    # Here we assume the face is the front face, so we offset by depth / 2
    cuboid_center_offset = np.array([0, 0, -d])
    
    # Convert rvec to rotation matrix
    R, _ = cv.Rodrigues(rvec)
    
    # print(f" Reversed : {R}")

    # Create a matrix to flip the z-axis
    flip_z_axis = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ], dtype=np.float32)

    # Apply the z-axis flip matrix to the rotation matrix
    R_flipped_z = np.dot(R, flip_z_axis)

    # Rotate the offset using the modified rotation matrix
    rotated_offset = np.dot(cuboid_center_offset, R_flipped_z.T)
    
    # Calculate the cuboid center in world coordinates
    cuboid_center = face_center + rotated_offset

    # Apply the rotation to the local vertices using the modified rotation matrix
    rotated_vertices = np.dot(local_vertices, R_flipped_z.T)

    # Translate the vertices to the cuboid center point
    z_reversed_vertices = rotated_vertices + cuboid_center

    return z_reversed_vertices

def publish_cuboid_marker(vertices, marker_id, marker_array_msg_black, obj_id):
    marker = Marker()
    marker.header.frame_id = "camera_color_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cuboids"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    # Define the orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.005
    marker.color.a = 1.0

    # Define color based on marker ID
    color_map = {
        1: (1.0, 0.0, 0.0),  # Red
        2: (1.0, 0.5, 0.0),  # Orange
        3: (1.0, 1.0, 0.0),  # Yellow
        4: (0.0, 1.0, 0.0),  # Green
        5: (0.0, 0.0, 1.0),  # Blue
        6: (0.29, 0.0, 0.51),  # Indigo
        7: (0.56, 0.0, 1.0),  # Violet
        8: (1.0, 0.75, 0.8)   # Pink
    }

    if marker_id in color_map:
        marker.color.r, marker.color.g, marker.color.b = color_map[obj_id+1]
    else:
        marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0  # Default to white if ID not in map

    # Define the cuboid edges
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    for start, end in edges:
        marker.points.append(Point(vertices[start][0], vertices[start][1], vertices[start][2]))
        marker.points.append(Point(vertices[end][0], vertices[end][1], vertices[end][2]))

    marker_array_msg_black.markers.append(marker)

def find_optimal_clusters(data, max_k):
    # Ensure that max_k does not exceed the number of samples
    max_k = min(max_k, len(data) - 1)
    iters = range(2, max_k + 1)
    sse = []
    silhouette_scores = []

    for k in iters:
        kmeans = KMeans(n_clusters=k, n_init='auto')
        kmeans.fit(data)
        sse.append(kmeans.inertia_)

        if k >= 2 and len(data) > k:  # Ensure there are at least 2 clusters and enough data
            silhouette_avg = silhouette_score(data, kmeans.labels_)
            silhouette_scores.append(silhouette_avg)
        else:
            silhouette_scores.append(float('nan'))

    # Handle the case where no valid silhouette score was computed
    if not silhouette_scores or all(np.isnan(silhouette_scores)):
        best_k = 2 if len(data) > 2 else 1
    else:
        best_k = iters[silhouette_scores.index(max(silhouette_scores))]

    return iters, sse, silhouette_scores, best_k

def objects_to_robot_coordinate(combined_objects):
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
    # print(f"camera2base : \n {camera2base}")      #mm단위
    # Extract rotation matrix (top-left 3x3) from camera2base
    rotation_camera2base = camera2base[:3, :3]
    # Extract translation vector (top-right 3x1) from camera2base
    translation_camera2base = camera2base[:3, 3]
    
    for obj_name, obj_data in combined_objects.items():
        if obj_data != {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector': [], 'area' : []}:
            # Convert positions from camera to base coordinate system
            positions = np.array(obj_data['face_center_3d'])
            positions = positions * 1000
            positions_base = np.dot(rotation_camera2base, positions.T).T + translation_camera2base
            # print(f" positions after transformation : {positions_base}")
            obj_data['face_center_3d'] = positions_base.tolist()            


            normals = np.array(obj_data['normal_vector'])
            normals_base = np.dot(rotation_camera2base, normals.T).T
            obj_data['normal_vector'] = normals_base.tolist()

            rvecs = np.array(obj_data['corrected_rotation_vector'])
            rvecs = rvecs.reshape(len(rvecs), -1)
            rotation_matrices = np.array([cv.Rodrigues(rvec)[0] for rvec in rvecs])
            transformed_rotation_matrices = np.matmul(rotation_camera2base, rotation_matrices)
            new_rvecs = np.array([cv.Rodrigues(mat)[0] for mat in transformed_rotation_matrices])
            obj_data['corrected_rotation_vector'] = new_rvecs.tolist()
    return combined_objects

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

def process_combined_objects(combined_objects):
    obj_names = []
    final_points = []
    final_normals = []
    final_rvecs = []

    all_positions = []

    for obj_name, obj_data in combined_objects.items():
        if obj_data != {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector': [], 'area' : []}:
            positions = np.array(obj_data['face_center_3d'])
            #positions = positions * 100
            all_positions.append(positions)  # Collecting all positions to find global min and max later

    # Combine all positions to find global min and max
    all_positions_combined = np.vstack(all_positions)
    x_min, y_min, z_min = np.min(all_positions_combined, axis=0)
    x_max, y_max, z_max = np.max(all_positions_combined, axis=0)

    # # Set up 2x5 subplots
    # fig, axs = plt.subplots(2, 5, figsize=(25, 10), subplot_kw={'projection': '3d'})
    # plot_index = 0

    for obj_name, obj_data in combined_objects.items():
        if obj_data != {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector': [], 'area': []}:
            positions = np.array(obj_data['face_center_3d'])
            normals = np.array(obj_data['normal_vector'])
            rvecs = np.array(obj_data['corrected_rotation_vector'])
            rvecs = rvecs.reshape(len(rvecs), -1)
            areas = np.array(obj_data['area'])

            if len(areas) > 0:  # Ensure areas are not empty
                # 면적 필터링
                area_threshold = np.percentile(areas, 40)  # 면적의 하위 40% 필터링
                valid_area_indices = areas > area_threshold

                # 면적 기준으로 데이터 분리
                survived_areas = areas[valid_area_indices]
                survived_positions = positions[valid_area_indices]
                survived_normals = normals[valid_area_indices]
                survived_rvecs = rvecs[valid_area_indices]
                filtered_areas = areas[~valid_area_indices]  # 제거된 면적들

                # Z축 벡터의 마지막 원소 값 필터링
                valid_z_indices = []
                survived_z_vectors = []
                filtered_z_vectors = []

                for idx, rvec in enumerate(survived_rvecs):
                    rotation_matrix, _ = cv.Rodrigues(rvec)
                    z_axis = rotation_matrix[:, 2]  # Z축 벡터
                    if z_axis[2] > 0:  # Z축의 마지막 원소가 양수인 경우
                        valid_z_indices.append(idx)
                        survived_z_vectors.append(z_axis)
                    else:
                        filtered_z_vectors.append(z_axis)

                # survived_z_vectors와 filtered_z_vectors를 numpy 배열로 변환
                survived_z_vectors = np.array(survived_z_vectors)
                filtered_z_vectors = np.array(filtered_z_vectors)

                # Z축 기준으로 최종 데이터 업데이트
                survived_positions = survived_positions[valid_z_indices]
                survived_normals = survived_normals[valid_z_indices]
                survived_rvecs = survived_rvecs[valid_z_indices]
                survived_areas = survived_areas[valid_z_indices]
                
                positions = survived_positions
                normals = survived_normals
                rvecs = survived_rvecs
                areas = survived_areas
                
                
                # 출력
                print(f"Object Name: {obj_name}")
                print("Survived Data:")
                print(f"- Survived Z-Axis Vectors:\n{survived_z_vectors}")
                print(f"- Survived Areas:\n{survived_areas}")
                print("-" * 50)
                print("Filtered Data (Did Not Survive):")
                print(f"- Filtered Z-Axis Vectors:\n{filtered_z_vectors}")
                print(f"- Filtered Areas:\n{filtered_areas}")
                print("=" * 50)

                    

            if len(positions) <= 3:
                print(f"No sufficient position data available for {obj_name}")
                continue

            max_k = 10
            # print(f"positions : {positions}")
            iters, sse, silhouette_scores, best_k = find_optimal_clusters(positions, max_k)

            kmeans = KMeans(n_clusters=best_k)
            kmeans_labels = kmeans.fit_predict(positions)
            combined_objects[obj_name]['kmeans_labels'] = kmeans_labels

            hierarchical = AgglomerativeClustering(n_clusters=best_k)
            hierarchical_labels = hierarchical.fit_predict(positions)
            combined_objects[obj_name]['hierarchical_labels'] = hierarchical_labels

            def most_frequent_label(labels):
                unique, counts = np.unique(labels, return_counts=True)
                return unique[np.argmax(counts)]

            most_frequent_kmeans_label = most_frequent_label(kmeans_labels)
            centroid_kmeans = np.mean(positions[kmeans_labels == most_frequent_kmeans_label], axis=0)
            distances_kmeans = np.linalg.norm(positions[kmeans_labels == most_frequent_kmeans_label] - centroid_kmeans, axis=1)

            z_scores_kmeans = zscore(distances_kmeans)
            filtered_indices_kmeans = np.abs(z_scores_kmeans) < 3

            kmeans_filtered_positions = positions[kmeans_labels == most_frequent_kmeans_label][filtered_indices_kmeans]
            kmeans_filtered_normals = normals[kmeans_labels == most_frequent_kmeans_label][filtered_indices_kmeans]
            kmeans_filtered_rvecs = rvecs[kmeans_labels == most_frequent_kmeans_label][filtered_indices_kmeans]

            def find_closest_to_centroid(positions, centroid):
                distances = np.linalg.norm(positions - centroid, axis=1)
                return positions[np.argmin(distances)], np.argmin(distances)

            if len(kmeans_filtered_positions) > 3:
                additional_kmeans = KMeans(n_clusters=min(5, len(kmeans_filtered_positions)))
                additional_kmeans_labels = additional_kmeans.fit_predict(kmeans_filtered_positions)
                most_frequent_additional_kmeans_label = most_frequent_label(additional_kmeans_labels)
                kmeans_filtered_filtered_positions = kmeans_filtered_positions[additional_kmeans_labels == most_frequent_additional_kmeans_label]
                kmeans_filtered_filtered_normals = kmeans_filtered_normals[additional_kmeans_labels == most_frequent_additional_kmeans_label]
                kmeans_filtered_filtered_rvecs = kmeans_filtered_rvecs[additional_kmeans_labels == most_frequent_additional_kmeans_label]
            else:
                kmeans_filtered_filtered_positions = kmeans_filtered_positions
                kmeans_filtered_filtered_normals = kmeans_filtered_normals
                kmeans_filtered_filtered_rvecs = kmeans_filtered_rvecs

            final_kmeans_centroid = np.mean(kmeans_filtered_filtered_positions, axis=0) if len(kmeans_filtered_filtered_positions) > 0 else None

            if final_kmeans_centroid is not None:
                central_kmeans_position, central_kmeans_index = find_closest_to_centroid(kmeans_filtered_filtered_positions, final_kmeans_centroid)
                central_kmeans_normal = kmeans_filtered_filtered_normals[central_kmeans_index]
                central_kmeans_rvec = kmeans_filtered_filtered_rvecs[central_kmeans_index]

                final_points.append(central_kmeans_position)
                final_normals.append(central_kmeans_normal)
                final_rvecs.append(central_kmeans_rvec)
                obj_names.append(obj_name)
            else:
                central_kmeans_position = centroid_kmeans
                central_kmeans_normal = normals[kmeans_labels == most_frequent_kmeans_label][0]
                central_kmeans_rvec = rvecs[kmeans_labels == most_frequent_kmeans_label][0]
                final_points.append(central_kmeans_position)
                final_normals.append(central_kmeans_normal)
                final_rvecs.append(central_kmeans_rvec)
                obj_names.append(obj_name)


            # # Plotting the K-Means clusters in 3D space on the corresponding subplot with fixed axis limits
            # ax_fixed = axs[0, plot_index]
            # ax_fixed.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c=kmeans_labels, cmap='viridis', marker='o', s=50)
            # ax_fixed.scatter(centroid_kmeans[0], centroid_kmeans[1], centroid_kmeans[2], c='red', marker='x', s=100, label='Centroid')
            # ax_fixed.set_title(f'{obj_name} (Fixed)')
            # ax_fixed.set_xlabel('X')
            # ax_fixed.set_ylabel('Y')
            # ax_fixed.set_zlabel('Z')
            # ax_fixed.set_xlim([x_min, x_max])
            # ax_fixed.set_ylim([y_min, y_max])
            # ax_fixed.set_zlim([z_min, z_max])

            # Plotting the K-Means clusters in 3D space on the corresponding subplot with individual axis limits
            # ax_individual = axs[1, plot_index]
            # ax_individual.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c=kmeans_labels, cmap='viridis', marker='o', s=50)
            # ax_individual.scatter(centroid_kmeans[0], centroid_kmeans[1], centroid_kmeans[2], c='red', marker='x', s=100, label='Centroid')
            # ax_individual.set_title(f'{obj_name} (Individual)')
            # ax_individual.set_xlabel('X')
            # ax_individual.set_ylabel('Y')
            # ax_individual.set_zlabel('Z')

            # plot_index += 1
            # if plot_index >= 5:
            #     break  # We only want to plot 5 objects

    plt.tight_layout()
    # plt.show()

    return final_points, final_normals, final_rvecs, obj_names

def create_pose_matrix(point, rvec):
    # Convert rvec to rotation matrix
    rotation_matrix, _ = cv.Rodrigues(rvec)
    
    # Create a 4x4 transformation matrix
    pose_matrix = np.eye(4)
    pose_matrix[0:3, 0:3] = rotation_matrix  # Set rotation part
    pose_matrix[0:3, 3] = point  # Set translation part (point)

    return pose_matrix

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

def publish_un_icp_point_cloud(o3d_point_cloud, label):
    # Convert Open3D PointCloud to NumPy array
    points_np = np.asarray(o3d_point_cloud.points)
    
    # Assign a color (e.g., red) to all points
    color = [255, 0, 0]  # RGB values for red

    if points_np.size > 0:  # Check if the NumPy array has points
        # Create header for PointCloud2
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_color_optical_frame'  # Set this to the appropriate frame_id for your setup
        
        # Combine points and colors
        points_with_color = []
        for point in points_np:
            r, g, b = color
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]  # Combine RGB values into a single integer
            points_with_color.append((point[0], point[1], point[2], rgb))
        
        # Define the fields for the PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]
        
        # Create PointCloud2 message
        point_cloud_msg = point_cloud2.create_cloud(header, fields, points_with_color)
        
        # Publish PointCloud2 message
        if label == 'color_logo':
            point_cloud_pub_color_logo2.publish(point_cloud_msg)
        elif label == 'black_logo':
            point_cloud_pub_black_logo2.publish(point_cloud_msg)
        # elif label == 'round_logo':
        #     point_cloud_pub_round_logo2.publish(point_cloud_msg)
        elif label == 'blue_robot':
            point_cloud_pub_blue_robot2.publish(point_cloud_msg)
        elif label == 'red_robot':
            point_cloud_pub_red_robot2.publish(point_cloud_msg)
        elif label == 'round_logo':    
            point_cloud_pub_round_logo2.publish(point_cloud_msg)

def publish_point_cloud(o3d_point_cloud, label):
    # Convert Open3D PointCloud to NumPy array
    points_np = np.asarray(o3d_point_cloud.points)
    
    # Define colors for each label
    label_colors = {
        'color_logo': [255, 255, 255],  # 하양
        'black_logo': [0, 0, 0],    # 검정
        'round_logo': [255, 165, 0],  # 주황
        'blue_robot': [0, 0, 255],  # 파랑
        'red_robot': [255, 0, 0],    # 빨강
        'roi' : [255, 192, 203] # 핑크
    }
    
    # Default color if label is not found
    default_color = [255, 255, 255]  # White
    color = label_colors.get(label, default_color)
    
    if points_np.size > 0:  # Check if the  NumPy array has points
        # Create header for PointCloud2
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_color_optical_frame'  # Set this to the appropriate frame_id for your setup
        
        # Combine points and colors
        points_with_color = []
        for point in points_np:
            r, g, b = color
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]  # Combine RGB values into a single integer
            points_with_color.append((point[0], point[1], point[2], rgb))
        
        # Define the fields for the PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
        ]
        
        # Create PointCloud2 message
        point_cloud_msg = point_cloud2.create_cloud(header, fields, points_with_color)
        
        # Publish PointCloud2 message
        if label == 'color_logo':
            point_cloud_pub_color_logo.publish(point_cloud_msg)
        elif label == 'black_logo':
            point_cloud_pub_black_logo.publish(point_cloud_msg)
        elif label == 'round_logo':
            point_cloud_pub_round_logo.publish(point_cloud_msg)
        elif label == 'blue_robot':
            point_cloud_pub_blue_robot.publish(point_cloud_msg)
        elif label == 'red_robot':
            point_cloud_pub_red_robot.publish(point_cloud_msg)
        else:
            rospy.logwarn(f"Unknown label: {label}. No publisher for this label.")

def scale_point_cloud(pcd, scale_factor):
    # Create a scaling matrix
    scaling_matrix = np.eye(4)
    scaling_matrix[0, 0] = scale_factor  # Scale x-axis
    scaling_matrix[1, 1] = scale_factor  # Scale y-axis
    scaling_matrix[2, 2] = scale_factor  # Scale z-axis
    
    # Apply scaling transformation
    pcd.transform(scaling_matrix)
    return pcd

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

def visualize_point_clouds(pcd1, pcd2, axis_lines, output_ply_path="결과물/p2plane/3views/회차.ply"):
    # Create a visualizer objecㅅ
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

    # Save the combined point cloud as a PLY file
    o3d.io.write_point_cloud(output_ply_path, combined_pcd)
    
    # Close the visualizer
    vis.destroy_window()

def combine_point_clouds(point_clouds):
    combined_cloud = o3d.geometry.PointCloud()
    
    for pc in point_clouds:
        combined_cloud.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(combined_cloud.points), np.asarray(pc.points))))
    
    return combined_cloud

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

def publish_multiple_cuboids(obj_names, final_points, final_rvecs):
    # Create a MarkerArray message
    marker_array_msg = MarkerArray()

    # Lists to store fitness results for each cuboid
    fitness_before_icp = []
    fitness_after_icp = []
    names = []
    vertices_before_transformation = []
    vertices_test = []
    transformed_vertices_ = []
    
    # Initialize total_cuboid_pointcloud as a list
    total_cuboid_pointcloud = []

    # Iterate through each cuboid's parameters
    for i, (name, point, rvec) in enumerate(zip(obj_names, final_points, final_rvecs)):
        # Define cuboid dimensions
        width = 0.059
        height = 0.059
        depth = 0.059
        
        # Get the vertices for the cuboid
        point = point * 0.01
        pose_matrix = create_pose_matrix(point, rvec)

        vertices = get_cuboid_vertices_from_face_center(point, rvec, width, height, depth)
        z_reversed_vertices = get_z_reversed_cuboid_vertices_from_face_center(point, rvec, width, height, depth)
        vertices_100_larger = vertices * 1000
        z_reversed_vertices_larger = z_reversed_vertices * 1000


        trans_init = np.eye(4)  # Initial transformation matrix

        print(name)

        def load_and_merge_point_clouds(file_names):
            # Initialize an empty list to hold the point clouds
            pcd_list = []

            # Load each point cloud and add it to the list
            for file_name in file_names:
                pcd = o3d.io.read_point_cloud(file_name)
                pcd_list.append(pcd)
            
            # Merge all point clouds into one
            merged_pcd = o3d.geometry.PointCloud()
            for pcd in pcd_list:
                merged_pcd += pcd
            
            return merged_pcd

        def remove_noise(pcd, nb_neighbors=20, std_ratio=2.0):
            # Print the number of points before noise removal
            num_points_before_noise_removal = len(pcd.points)
            # print(f"Number of points before noise removal: {num_points_before_noise_removal}")

            # Apply statistical outlier removal to remove noise
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
            cleaned_pcd = pcd.select_by_index(ind)
            
            # Print the number of points after noise removal
            num_points_after_noise_removal = len(cleaned_pcd.points)
            # print(f"Number of points after noise removal: {num_points_after_noise_removal}")
            
            return cleaned_pcd

        def remove_duplicates(pcd, voxel_size=0.004):
            # Print the number of points before removing duplicates
            num_points_before_downsampling = len(pcd.points)
            # print(f"Number of points before downsampling: {num_points_before_downsampling}")

            # Apply voxel downsampling to remove duplicate points
            downsampled_pcd = pcd.voxel_down_sample(voxel_size)

            # Print the number of points after removing duplicates
            num_points_after_downsampling = len(downsampled_pcd.points)
            # print(f"Number of points after downsampling: {num_points_after_downsampling}")
            
            return downsampled_pcd

        def visualize_point_cloud(pcd):
            # Create a visualizer object
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            
            # Add the point cloud to the visualizer
            vis.add_geometry(pcd)
            
            # Optionally, set a view control for better visualization
            ctr = vis.get_view_control()
            ctr.set_zoom(0.8)
            
            # Run the visualizer
            vis.run()
            
            # Destroy the visualizer window after use
            vis.destroy_window()

        def save_point_cloud(pcd, file_name):
            # Save the processed point cloud to a file
            o3d.io.write_point_cloud(file_name, pcd)
            # print(f"Point cloud saved to {file_name}")

        file_names = [
            "PointCloudFromCircleRobot/transformed_pointcloud_view_1.ply",
            "PointCloudFromCircleRobot/transformed_pointcloud_view_2.ply",
            "PointCloudFromCircleRobot/transformed_pointcloud_view_3.ply",
            "PointCloudFromCircleRobot/transformed_pointcloud_view_4.ply"
        ]

        # file_names = [
        #     "PointCloudFromCircleRobot/pointcloud_view_1.ply",
        #     "PointCloudFromCircleRobot/pointcloud_view_2.ply",
        #     "PointCloudFromCircleRobot/pointcloud_view_3.ply",
        #     "PointCloudFromCircleRobot/pointcloud_view_4.ply"
        # ]

        # Load and merge point clouds
        merged_pcd = load_and_merge_point_clouds(file_names)

        # Remove noise and then duplicates
        cleaned_pcd = remove_noise(merged_pcd)
        final_pcd = remove_duplicates(cleaned_pcd)

        # Save the final processed point cloud
        save_point_cloud(final_pcd, "PointCloudFromCircleRobot/map.ply")
        
        # Load the point cloud
        file_name = "PointCloudFromCircleRobot/map.ply"
        pointcloud_map = o3d.io.read_point_cloud(file_name)

        # Convert to numpy array
        points = np.asarray(pointcloud_map.points)

        # Perform initial DBSCAN clustering
        dbscan = DBSCAN(eps=0.01, min_samples=5)
        labels = dbscan.fit_predict(points)

        # Print unique labels and their count
        unique_labels = np.unique(labels)
        num_clusters = len(unique_labels) - (1 if -1 in unique_labels else 0)

        # print(f"Unique labels (including noise): {unique_labels}")
        # print(f"Number of clusters (excluding noise): {num_clusters}")

        # Create a mask to keep points
        keep_mask = np.ones(len(points), dtype=bool)

        # Process each cluster
        for label in unique_labels:
            if label == -1:  # Skip noise
                continue

            # Get points in the current cluster
            cluster_points = points[labels == label]

            # Check the size of the cluster
            if len(cluster_points) > 1000:  # Only process if larger than 1000
                # Calculate how many points to remove (30% of the cluster)
                num_to_remove = int(len(cluster_points) * 0.35)

                # Calcu late distances from the centroid
                centroid = np.mean(cluster_points, axis=0)
                distances = np.linalg.norm(cluster_points - centroid, axis=1)

                # Get indices of the farthest points to remove
                farthest_indices = np.argsort(distances)[-num_to_remove:]

                # Create a mask to keep points that are not the farthest
                cluster_mask = np.ones(len(cluster_points), dtype=bool)
                cluster_mask[farthest_indices] = False

                # Update the overall keep_mask
                keep_mask[labels == label] = cluster_mask

        # Filter points based on keep_mask
        filtered_points = points[keep_mask]
        filtered_labels = labels[keep_mask]

        # Create a color array for filtered clusters
        colored_points = np.zeros((filtered_points.shape[0], 3))
        for i, label in enumerate(np.unique(filtered_labels)):
            if label == -1:  # Noise points
                colored_points[filtered_labels == label] = np.array([0, 0, 0])  # Black for noise
            else:
                color = plt.cm.hsv(i / len(np.unique(filtered_labels)))[:3]  # New colors for clusters
                colored_points[filtered_labels == label] = color

        # Update pointcloud_map with filtered points and colors
        pointcloud_map.points = o3d.utility.Vector3dVector(filtered_points)
        pointcloud_map.colors = o3d.utility.Vector3dVector(colored_points)
        
        #o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

        icp_convergence_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-8, relative_rmse=1e-8, max_iteration=100)

        z_reversed_synthetic_points = sample_points_on_cuboid(z_reversed_vertices, num_points_per_face=100)
        synthetic_points = sample_points_on_cuboid(vertices, num_points_per_face=100)

        axis_lines = create_axis_lines(scale=0.1)
        scale_factor = 1
        threshold = 0.1
        synthetic_points = scale_point_cloud(synthetic_points, scale_factor)
        z_reversed_synthetic_points = scale_point_cloud(z_reversed_synthetic_points, scale_factor)
        pointcloud_map = scale_point_cloud(pointcloud_map, scale_factor*10)

        def estimate_normals(point_cloud):
            point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # 원본 포인트 클라우드와 변환된 포인트 클라우드의 법선 추정
        estimate_normals(pointcloud_map)
        estimate_normals(synthetic_points)
        estimate_normals(z_reversed_synthetic_points)

        # ICP 실행 (Point-to-Plane)
        #print("Un Flipped")
        icp_result = o3d.pipelines.registration.registration_icp(
            synthetic_points, pointcloud_map, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(), icp_convergence_criteria)

        #print("Flipped")
        flipped_icp_result = o3d.pipelines.registration.registration_icp(
            z_reversed_synthetic_points, pointcloud_map, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(), icp_convergence_criteria)

        transformation = icp_result.transformation
        #print(f"transformation : {transformation}")
        flipped_transformation = flipped_icp_result.transformation
        synthetic_points.transform(transformation)
        z_reversed_synthetic_points.transform(flipped_transformation)
        
        
        fitness = None
        if icp_result.fitness > flipped_icp_result.fitness:
            
            print("Un Flipped")
            total_cuboid_pointcloud.append(synthetic_points)
            print(f"unflipped : {icp_result.fitness}")
            print(f"flipped : {flipped_icp_result.fitness}")
            
            fitness = icp_result.fitness
            
        else :
            print("Flipped")
            total_cuboid_pointcloud.append(z_reversed_synthetic_points)
            print(f"unflipped : {icp_result.fitness}")
            print(f"flipped : {flipped_icp_result.fitness}")
            fitness = flipped_icp_result.fitness
            vertices_100_larger = z_reversed_vertices_larger

        vertices_from_synthetic_points = extract_vertices_from_synthetic_points(synthetic_points)
        transformed_vertices = np.dot(np.hstack((vertices_100_larger, np.ones((8, 1)))), transformation.T)[:, :3]


        # Store fitness after ICP
        icp_fitness = icp_result.fitness
        fitness_after_icp.append(icp_fitness)

        # Scale back the point cloud after ICP
        synthetic_points = scale_point_cloud(synthetic_points, 1 / scale_factor)
        publish_point_cloud(synthetic_points, name)
        #print(transformed_vertices)
        
        scale = 1000

        scaled_transformation = transformation.copy()
        # translation 부분을 배율만큼 곱함
        scaled_transformation[0:3, 3] *= scale  
        
        # 변환된 vertices_100_larger 계산
        vertices_homogeneous = np.hstack((vertices_100_larger, np.ones((vertices_100_larger.shape[0], 1))))  # 동차 좌표로 변환
        transformed_vertices = scaled_transformation @ vertices_homogeneous.T  # 변환 행렬과 곱함
        transformed_vertices = transformed_vertices.T[:, :3]  # 변환된 결과에서 x, y, z 좌표만 추출

        #print(f"Transformed vertices (scale x{scale}):\n{transformed_vertices}\n")

        vertices_from_synthetic_points = transformed_vertices * 0.1
        
        if fitness > 0.4:
            transformed_vertices_.append(vertices_from_synthetic_points)
            vertices_before_transformation.append(vertices_100_larger)
        #print(f"vertices after icp : {vertices_from_synthetic_points}")
        #print(f"vertices before icp : {vertices_100_larger}")
        # v_test = vertices_100_larger.transform
        # vertices_test.append()

            names.append(name)

        # Create a marker for this cuboid
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cuboids"
        marker.id = i
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.005
        marker.color.a = 1.0

        # Define color based on marker ID
        color_map = {
            1: (1.0, 0.0, 0.0),  # Red
            2: (1.0, 0.5, 0.0),  # Orange
            3: (1.0, 1.0, 0.0),  # Yellow
            4: (0.0, 1.0, 0.0),  # Green
            5: (0.0, 0.0, 1.0),  # Blue
            6: (0.29, 0.0, 0.51),  # Indigo
            7: (0.56, 0.0, 1.0),  # Violet
            8: (1.0, 0.75, 0.8)   # Pink
        }

        marker.color.r, marker.color.g, marker.color.b = color_map.get(i + 1, (1.0, 1.0, 1.0))  # Default to white if ID not in map

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        for start, end in edges:
            marker.points.append(Point(vertices[start][0], vertices[start][1], vertices[start][2]))
            marker.points.append(Point(vertices[end][0], vertices[end][1], vertices[end][2]))

        marker_array_msg.markers.append(marker)
    # Combine all cuboid point clouds into one
    combined_cuboid_cloud = combine_point_clouds(total_cuboid_pointcloud)

    # Visualize the original point cloud and the combined cuboid point cloud
    visualize_point_clouds(pointcloud_map, combined_cuboid_cloud, axis_lines)

    multi_cuboid_pub.publish(marker_array_msg)

    # Print fitness results before and after ICP
    # print("Fitness results for each cuboid:")
    # for i, (fitness_b, fitness_a) in enumerate(zip(fitness_before_icp, fitness_after_icp)):
    #     print(f"Cuboid {i+1} ({obj_names[i]}): Fitness before ICP = {fitness_b}, Fitness after ICP = {fitness_a}")

    # Find the highest fitness value
    max_fitness_index = fitness_after_icp.index(max(fitness_after_icp))  # Index of the highest fitness value
    max_fitness_name = obj_names[max_fitness_index]  # Name of the object with the highest fitness
    max_fitness_value = fitness_after_icp[max_fitness_index]  # Highest fitness value

    # print(f"Fitness가 가장 높은 것은 {max_fitness_name}입니다. Fitness 값 = {max_fitness_value}")

    # Find the lowest fitness value
    min_fitness_index = fitness_after_icp.index(min(fitness_after_icp))  # Index of the lowest fitness value
    min_fitness_name = obj_names[min_fitness_index]  # Name of the object with the lowest fitness
    min_fitness_value = fitness_after_icp[min_fitness_index]  # Lowest fitness value

    # print(f"Fitness가 가장 낮은 것은 {min_fitness_name}입니다. Fitness 값 = {min_fitness_value}")

    return vertices_before_transformation, transformed_vertices_, names

def compute_add(transformed_vertices, true_vertices):
    # true_vertices로 KDTree 생성
    tree = KDTree(true_vertices)
    
    # 각 transformed_vertices 점에 대해 가장 가까운 true_vertices 점 찾기
    distances, _ = tree.query(transformed_vertices)

    # 평균 거리 반환
    return np.mean(distances)

def compute_pose_matrix(vertices):
    # 중점 계산
    centroid = np.mean(vertices, axis=0)

    # 각 축의 방향 계산
    x_axis = (vertices[1] - vertices[0]) / np.linalg.norm(vertices[1] - vertices[0])  # X축 방향
    y_axis = (vertices[2] - vertices[0]) / np.linalg.norm(vertices[2] - vertices[0])  # Y축 방향
    z_axis = np.cross(x_axis, y_axis)  # Z축은 X와 Y의 외적

    # X, Y, Z 축 정규화
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)  # Y축 재계산 (정규 직교화)

    # 회전 행렬 생성
    rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
    # print(f"rotation_matrix : {rotation_matrix}")
    # 4x4 변환 행렬 생성
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix  # 회전 행렬
    pose_matrix[:3, 3] = centroid  # 중점 위치

    return pose_matrix

def merge_dicts(dicts):
    merged = {}
    for d in dicts:
        for key, value in d.items():
            if key not in merged:
                merged[key] = {k: [] for k in value.keys()}  # Initialize empty lists
            for sub_key in value:
                merged[key][sub_key].extend(value[sub_key])
    return merged

def polygon_area(points):
    x = points[:, 0]
    y = points[:, 1]
    
    # 면적 공식 적용
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

def rgb_msg_callback(msg):
    global name_list, label_dict, dist_coeffs, objects, frame_count, combined_objects,flag, angle_count
    global combined_objects_1, combined_objects_2, combined_objects_3, combined_objects_4
    global cloud_from_view_1, cloud_from_view_2, cloud_from_view_3, cloud_from_view_4
    
    if frame_count == 0:
        # Initialize the combined_objects dictionary at the start of the first frame
        combined_objects = {
            'color_logo': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector' : [], 'area' : []},
            'black_logo': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector' : [], 'area' : []},
            'round_logo': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector' : [], 'area' : []},
            'blue_robot': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector' : [], 'area' : []},
            'red_robot': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d': [], 'vertices': [], 'face_center_3d': [], 'corrected_rotation_vector': [], 'normal_vector' : [], 'area' : []}
        }
        
    objects = {
        'color_logo': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d':[], 'vertices' : [], 'face_center_3d' : [], 'corrected_rotation_vector':[], 'normal_vector' : [], 'area' : []},
        'black_logo': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d':[], 'vertices' : [],'face_center_3d' : [],'corrected_rotation_vector':[], 'normal_vector' : [], 'area' : []},
        'round_logo': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d':[], 'vertices' : [],'face_center_3d' : [],'corrected_rotation_vector':[], 'normal_vector' : [], 'area' : []},
        'blue_robot': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d':[], 'vertices' : [],'face_center_3d' : [],'corrected_rotation_vector':[], 'normal_vector' : [], 'area' : []},
        'red_robot': {'label': [], 'rvec': [], 'position': [], 'keypoints_2d':[], 'vertices' : [],'face_center_3d' : [],'corrected_rotation_vector':[], 'normal_vector' : [], 'area' : []}
    }
    try:
        # Convert ROS image message to OpenCV format
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run object detection
        results = model(image, verbose=False)
        

        names = []
        bboxes = []  # To store bounding boxes
        keypoints_data = []

        # Collect names, bounding boxes, and keypoints from the model results
        for result in results:
            boxes = result.boxes
            names.extend(boxes.cls.cpu().numpy())
            bboxes.extend(boxes.xyxy.cpu().numpy())

            for prediction in result:
                keypoints = prediction.keypoints.cpu().numpy()
                keypoints_data.append(keypoints)

        # Draw bounding boxes and labels
        for idx, bbox in enumerate(bboxes):
            x1, y1, x2, y2 = map(int, bbox)  # Convert to integer
            label_index = int(names[idx])  # Get the integer index from names
            if label_index < len(name_list):  # Ensure index is valid
                label = name_list[label_index]
            else:
                label = "Unknown"

            # Draw bounding box
            cv.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Put text
            text_size = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            text_x = x1
            text_y = y1 - 10
            if text_y < 10:
                text_y = y1 + text_size[1] + 10
            cv.putText(image, label, (text_x, text_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        # Process keypoints and draw 3D axes
        if len(keypoints_data) > 0:
            for idx in range(len(names)):  # Process all detected labels
                if len(keypoints_data) > idx:
                    image_points = keypoints_data[idx].data[0][:4]
                    if label == 'color_logo':
                        area = polygon_area(image_points)
                        random_points = random_points_inside_quadrilateral(image_points, 20)
                        random_points = [np.array(point).astype(int).tolist() for point in random_points]
                        r = []
                        for random_point in random_points:
                            random_point_3d = get_coordinate_depth(depth_image, random_point)
                            if random_point_3d.any() != 0:
                                r.append(random_point_3d)
                        if r != []:
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(np.array(r))

                            # Estimate plane and compute normal
                            if len(point_cloud.points) > 3:
                                # Compute plane using Open3D's estimate_normals
                                point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                                
                                # Use PCA or RANSAC to fit a plane to the point cloud
                                plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                                [a, b, c, d] = plane_model
                                
                                # The normal vector of the plane is (a, b, c)
                                normal_vector = np.array([-a, -b, -c])
                                normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the vector
                                origin = np.mean(np.array(r), axis=0)  # Use the mean of the points as the origin

                                # publish_point_cloud(point_cloud, label)

                                for point in random_points:
                                    point_2d = (int(point[0]), int(point[1]))  # 정수로 변환 후 사용
                                    cv.circle(image, point_2d, 5, (255, 0, 0), -1)  # 빨간색으로 특징점을 그립니다.

                                for point in image_points:
                                    point_2d = tuple(point.astype(int))
                                    cv.circle(image, point_2d, 5, (0, 255, 0), -1)  # 초록색으로 특징점을 그립니다.
                                
                                position_2d = image_points[0].astype(int)
                                position_3d = get_coordinate_depth(depth_image, position_2d)
                                x_value = position_3d[0]
                                y_value = position_3d[1]
                                z_value = position_3d[2]  # Extract z value

                                label_index = int(names[idx])  # Get the integer index from names
                                if label_index < len(name_list):  # Ensure index is valid
                                    label = name_list[label_index]
                                    if label in label_dict and image_points.shape[0] == 4:
                                        success, rotation_vector, translation_vector = cv.solvePnP(
                                            label_dict[label], image_points, camera_matrix, dist_coeffs
                                        )
                                    Rotm, _ = cv.Rodrigues(rotation_vector)
                                    corrected_rotation_vector, _ = cv.Rodrigues(align_z_axis_and_get_new_rotation_matrix(normal_vector, Rotm))

                                marker_array_msg_color = MarkerArray()
                                corrected_marker_array_msg_color = MarkerArray()
                                axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]])
                                imgpts, _ = cv.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

                                # Draw the axes on the frame
                                image = cv.drawFrameAxes(image, camera_matrix, dist_coeffs, rotation_vector, translation_vector, length=2)
                                face_center_2d = np.mean(image_points, axis=0).astype(int)
                                face_center_3d = get_coordinate_depth(depth_image, face_center_2d)
                                face_center_3d = face_center_3d * 0.01
                                width, height, depth = 0.065, 0.065, 0.065
                                tv = np.array([translation_vector[0][0], translation_vector[1][0], translation_vector[2][0]], dtype=np.float32) * 0.01
                                vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, rotation_vector, width, height, depth)
                                vertices_PnP = get_cuboid_vertices_from_face_center(tv, rotation_vector, width, height, depth)

                                corrected_vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, corrected_rotation_vector, width, height, depth)
                                corrected_vertices_PnP = get_cuboid_vertices_from_face_center(tv, corrected_rotation_vector, width, height, depth)

                                publish_cuboid_marker(vertices_measured, 1, marker_array_msg_color, 1)
                                publish_cuboid_marker(corrected_vertices_measured, 2, corrected_marker_array_msg_color, 2)
                            
                                for _ in range(len(names)):
                                    # Clear previous markers and publish new markers
                                    delete_marker = Marker()
                                    delete_marker.action = Marker.DELETEALL
                                    corrected_marker_array_msg_color.markers.insert(0, delete_marker)
                                    marker_array_msg_color.markers.insert(0, delete_marker)

                                if len(names) != 0:
                                    detection_msg = MarkerArray()
                                    detection_msg.markers = marker_array_msg_color.markers

                                # Update the objects dictionary
                                if label in objects:
                                    objects[label]['label'].append(label)
                                    objects[label]['rvec'].append(rotation_vector)
                                    objects[label]['position'].append(position_3d)
                                    objects[label]['vertices'].append(corrected_vertices_measured)
                                    objects[label]['keypoints_2d'].append(image_points)
                                    objects[label]['face_center_3d'].append(face_center_3d)
                                    objects[label]['corrected_rotation_vector'].append(corrected_rotation_vector)
                                    objects[label]['normal_vector'].append(normal_vector)
                                    objects[label]['area'].append(area)
                                    
                                               
                    if label == 'black_logo':
                        area = polygon_area(image_points)
                        random_points = random_points_inside_quadrilateral(image_points, 20)
                        random_points = [np.array(point).astype(int).tolist() for point in random_points]
                        r = []
                        for random_point in random_points:
                            random_point_3d = get_coordinate_depth(depth_image, random_point)
                            if random_point_3d.any() != 0:
                                r.append(random_point_3d)
                        if r != []:
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(np.array(r))

                            # Estimate plane and compute normal
                            if len(point_cloud.points) > 3:
                                # Compute plane using Open3D's estimate_normals
                                point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                                
                                # Use PCA or RANSAC to fit a plane to the point cloud
                                plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                                [a, b, c, d] = plane_model
                                
                                # The normal vector of the plane is (a, b, c)
                                normal_vector = np.array([-a, -b, -c])
                                normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the vector
                                origin = np.mean(np.array(r), axis=0)  # Use the mean of the points as the origin

                                # publish_point_cloud(point_cloud, label)

                                for point in random_points:
                                    point_2d = (int(point[0]), int(point[1]))  # 정수로 변환 후 사용
                                    cv.circle(image, point_2d, 5, (255, 0, 0), -1)  # 빨간색으로 특징점을 그립니다.

                                for point in image_points:
                                    point_2d = tuple(point.astype(int))
                                    cv.circle(image, point_2d, 5, (0, 255, 0), -1)  # 초록색으로 특징점을 그립니다.
                                
                                position_2d = image_points[0].astype(int)
                                position_3d = get_coordinate_depth(depth_image, position_2d)
                                x_value = position_3d[0]
                                y_value = position_3d[1]
                                z_value = position_3d[2]  # Extract z value

                                label_index = int(names[idx])  # Get the integer index from names
                                if label_index < len(name_list):  # Ensure index is valid
                                    label = name_list[label_index]
                                    if label in label_dict and image_points.shape[0] == 4:
                                        success, rotation_vector, translation_vector = cv.solvePnP(
                                            label_dict[label], image_points, camera_matrix, dist_coeffs
                                        )
                                    Rotm, _ = cv.Rodrigues(rotation_vector)
                                    corrected_rotation_vector, _ = cv.Rodrigues(align_z_axis_and_get_new_rotation_matrix(normal_vector, Rotm))
                                
                                marker_array_msg_black = MarkerArray()
                                corrected_marker_array_msg_black = MarkerArray()
                                axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]])
                                imgpts, _ = cv.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

                                # Draw the axes on the frame
                                image = cv.drawFrameAxes(image, camera_matrix, dist_coeffs, rotation_vector, translation_vector, length=2)
                                face_center_2d = np.mean(image_points, axis=0).astype(int)
                                face_center_3d = get_coordinate_depth(depth_image, face_center_2d)
                                face_center_3d = face_center_3d * 0.01
                                width, height, depth = 0.065, 0.065, 0.065
                                tv = np.array([translation_vector[0][0], translation_vector[1][0], translation_vector[2][0]], dtype=np.float32) * 0.01
                                vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, rotation_vector, width, height, depth)
                                vertices_PnP = get_cuboid_vertices_from_face_center(tv, rotation_vector, width, height, depth)

                                corrected_vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, corrected_rotation_vector, width, height, depth)
                                corrected_vertices_PnP = get_cuboid_vertices_from_face_center(tv, corrected_rotation_vector, width, height, depth)

                                publish_cuboid_marker(vertices_measured, 1, marker_array_msg_black, 1)
                                publish_cuboid_marker(corrected_vertices_measured, 2, corrected_marker_array_msg_black, 2)
                            
                                for _ in range(len(names)):
                                    # Clear previous markers and publish new markers
                                    delete_marker = Marker()
                                    delete_marker.action = Marker.DELETEALL
                                    corrected_marker_array_msg_black.markers.insert(0, delete_marker)
                                    marker_array_msg_black.markers.insert(0, delete_marker)

                                if len(names) != 0:
                                    detection_msg = MarkerArray()
                                    detection_msg.markers = marker_array_msg_black.markers

                                # Update the objects dictionary
                                if label in objects:
                                    objects[label]['label'].append(label)
                                    objects[label]['rvec'].append(rotation_vector)
                                    objects[label]['position'].append(position_3d)
                                    objects[label]['vertices'].append(corrected_vertices_measured)
                                    objects[label]['keypoints_2d'].append(image_points)
                                    objects[label]['face_center_3d'].append(face_center_3d)
                                    objects[label]['corrected_rotation_vector'].append(corrected_rotation_vector)
                                    objects[label]['normal_vector'].append(normal_vector)
                                    objects[label]['area'].append(area)
                        
                    if label == 'round_logo':
                        area = polygon_area(image_points)
                        random_points = random_points_inside_quadrilateral(image_points, 20)
                        random_points = [np.array(point).astype(int).tolist() for point in random_points]
                        r = []
                        for random_point in random_points:
                            random_point_3d = get_coordinate_depth(depth_image, random_point)
                            if random_point_3d.any() != 0:
                                r.append(random_point_3d)
                        if r != []:
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(np.array(r))

                            # Estimate plane and compute normal
                            if len(point_cloud.points) > 3:
                                # Compute plane using Open3D's estimate_normals
                                point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                                
                                # Use PCA or RANSAC to fit a plane to the point cloud
                                plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                                [a, b, c, d] = plane_model
                                
                                # The normal vector of the plane is (a, b, c)
                                normal_vector = np.array([-a, -b, -c])
                                normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the vector
                                origin = np.mean(np.array(r), axis=0)  # Use the mean of the points as the origin

                                # publish_point_cloud(point_cloud, label)

                                for point in random_points:
                                    point_2d = (int(point[0]), int(point[1]))  # 정수로 변환 후 사용
                                    cv.circle(image, point_2d, 5, (255, 0, 0), -1)  # 빨간색으로 특징점을 그립니다.

                                for point in image_points:
                                    point_2d = tuple(point.astype(int))
                                    cv.circle(image, point_2d, 5, (0, 255, 0), -1)  # 초록색으로 특징점을 그립니다.
                                
                                position_2d = image_points[0].astype(int)
                                position_3d = get_coordinate_depth(depth_image, position_2d)
                                x_value = position_3d[0]
                                y_value = position_3d[1]
                                z_value = position_3d[2]  # Extract z value

                                label_index = int(names[idx])  # Get the integer index from names
                                if label_index < len(name_list):  # Ensure index is valid
                                    label = name_list[label_index]
                                    if label in label_dict and image_points.shape[0] == 4:
                                        success, rotation_vector, translation_vector = cv.solvePnP(
                                            label_dict[label], image_points, camera_matrix, dist_coeffs
                                        )
                                    Rotm, _ = cv.Rodrigues(rotation_vector)
                                    corrected_rotation_vector, _ = cv.Rodrigues(align_z_axis_and_get_new_rotation_matrix(normal_vector, Rotm))
                                    

                                marker_array_msg_round = MarkerArray()
                                corrected_marker_array_msg_round = MarkerArray()
                                axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]])
                                imgpts, _ = cv.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

                                # Draw the axes on the frame
                                image = cv.drawFrameAxes(image, camera_matrix, dist_coeffs, rotation_vector, translation_vector, length=2)
                                face_center_2d = np.mean(image_points, axis=0).astype(int)
                                face_center_3d = get_coordinate_depth(depth_image, face_center_2d)
                                face_center_3d = face_center_3d * 0.01
                                width, height, depth = 0.065, 0.065, 0.065
                                tv = np.array([translation_vector[0][0], translation_vector[1][0], translation_vector[2][0]], dtype=np.float32) * 0.01
                                vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, rotation_vector, width, height, depth)
                                vertices_PnP = get_cuboid_vertices_from_face_center(tv, rotation_vector, width, height, depth)

                                corrected_vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, corrected_rotation_vector, width, height, depth)
                                corrected_vertices_PnP = get_cuboid_vertices_from_face_center(tv, corrected_rotation_vector, width, height, depth)

                                publish_cuboid_marker(vertices_measured, 1, marker_array_msg_round, 1)
                                publish_cuboid_marker(corrected_vertices_measured, 2, corrected_marker_array_msg_round, 2)
                            
                                for _ in range(len(names)):
                                    # Clear previous markers and publish new markers
                                    delete_marker = Marker()
                                    delete_marker.action = Marker.DELETEALL
                                    corrected_marker_array_msg_round.markers.insert(0, delete_marker)
                                    marker_array_msg_round.markers.insert(0, delete_marker)

                                if len(names) != 0:
                                    detection_msg = MarkerArray()
                                    detection_msg.markers = marker_array_msg_round.markers

                                # Update the objects dictionary
                                if label in objects:
                                    objects[label]['label'].append(label)
                                    objects[label]['rvec'].append(rotation_vector)
                                    objects[label]['position'].append(position_3d)
                                    objects[label]['vertices'].append(corrected_vertices_measured)
                                    objects[label]['keypoints_2d'].append(image_points)
                                    objects[label]['face_center_3d'].append(face_center_3d)
                                    objects[label]['corrected_rotation_vector'].append(corrected_rotation_vector)
                                    objects[label]['normal_vector'].append(normal_vector)
                                    objects[label]['area'].append(area)
                        
                    if label == 'blue_robot':
                        area = polygon_area(image_points)
                        random_points = random_points_inside_quadrilateral(image_points, 20)
                        random_points = [np.array(point).astype(int).tolist() for point in random_points]
                        r = []
                        for random_point in random_points:
                            random_point_3d = get_coordinate_depth(depth_image, random_point)
                            if random_point_3d.any() != 0:
                                r.append(random_point_3d)
                        if r != []:
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(np.array(r))

                            # Estimate plane and compute normal
                            if len(point_cloud.points) > 3:
                                # Compute plane using Open3D's estimate_normals
                                point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                                
                                # Use PCA or RANSAC to fit a plane to the point cloud
                                plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                                [a, b, c, d] = plane_model
                                
                                # The normal vector of the plane is (a, b, c)
                                normal_vector = np.array([-a, -b, -c])
                                normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the vector
                                origin = np.mean(np.array(r), axis=0)  # Use the mean of the points as the origin

                                # publish_point_cloud(point_cloud, label)

                                for point in random_points:
                                    point_2d = (int(point[0]), int(point[1]))  # 정수로 변환 후 사용
                                    cv.circle(image, point_2d, 5, (255, 0, 0), -1)  # 빨간색으로 특징점을 그립니다.

                                for point in image_points:
                                    point_2d = tuple(point.astype(int))
                                    cv.circle(image, point_2d, 5, (0, 255, 0), -1)  # 초록색으로 특징점을 그립니다.
                                
                                position_2d = image_points[0].astype(int)
                                position_3d = get_coordinate_depth(depth_image, position_2d)
                                x_value = position_3d[0]
                                y_value = position_3d[1]
                                z_value = position_3d[2]  # Extract z value

                                label_index = int(names[idx])  # Get the integer index from names
                                if label_index < len(name_list):  # Ensure index is valid
                                    label = name_list[label_index]
                                    if label in label_dict and image_points.shape[0] == 4:
                                        success, rotation_vector, translation_vector = cv.solvePnP(
                                            label_dict[label], image_points, camera_matrix, dist_coeffs
                                        )
                                    Rotm, _ = cv.Rodrigues(rotation_vector)
                                    corrected_rotation_vector, _ = cv.Rodrigues(align_z_axis_and_get_new_rotation_matrix(normal_vector, Rotm))
                                
                                marker_array_msg_blue_robot = MarkerArray()
                                corrected_marker_array_msg_blue_robot = MarkerArray()
                                axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]])
                                imgpts, _ = cv.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

                                # Draw the axes on the frame
                                image = cv.drawFrameAxes(image, camera_matrix, dist_coeffs, rotation_vector, translation_vector, length=2)
                                face_center_2d = np.mean(image_points, axis=0).astype(int)
                                face_center_3d = get_coordinate_depth(depth_image, face_center_2d)
                                face_center_3d = face_center_3d * 0.01
                                width, height, depth = 0.065, 0.065, 0.065
                                tv = np.array([translation_vector[0][0], translation_vector[1][0], translation_vector[2][0]], dtype=np.float32) * 0.01
                                vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, rotation_vector, width, height, depth)
                                vertices_PnP = get_cuboid_vertices_from_face_center(tv, rotation_vector, width, height, depth)

                                corrected_vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, corrected_rotation_vector, width, height, depth)
                                corrected_vertices_PnP = get_cuboid_vertices_from_face_center(tv, corrected_rotation_vector, width, height, depth)

                                publish_cuboid_marker(vertices_measured, 3, marker_array_msg_blue_robot, 3)
                                publish_cuboid_marker(corrected_vertices_measured, 6, corrected_marker_array_msg_blue_robot, 6)
                            
                                for _ in range(len(names)):
                                    # Clear previous markers and publish new markers
                                    delete_marker = Marker()
                                    delete_marker.action = Marker.DELETEALL
                                    corrected_marker_array_msg_blue_robot.markers.insert(0, delete_marker)
                                    marker_array_msg_blue_robot.markers.insert(0, delete_marker)

                                if len(names) != 0:
                                    detection_msg = MarkerArray()
                                    detection_msg.markers = marker_array_msg_blue_robot.markers

                                # Update the objects dictionary
                                if label in objects:
                                    objects[label]['label'].append(label)
                                    objects[label]['rvec'].append(rotation_vector)
                                    objects[label]['position'].append(position_3d)
                                    objects[label]['vertices'].append(corrected_vertices_measured)
                                    objects[label]['keypoints_2d'].append(image_points)
                                    objects[label]['face_center_3d'].append(face_center_3d)
                                    objects[label]['corrected_rotation_vector'].append(corrected_rotation_vector)
                                    objects[label]['normal_vector'].append(normal_vector)
                                    objects[label]['area'].append(area)
                        
                    if label == 'red_robot':
                        area = polygon_area(image_points)
                        random_points = random_points_inside_quadrilateral(image_points, 20)
                        random_points = [np.array(point).astype(int).tolist() for point in random_points]
                        r = []
                        for random_point in random_points:
                            random_point_3d = get_coordinate_depth(depth_image, random_point)
                            if random_point_3d.any() != 0:
                                r.append(random_point_3d)
                        if r != []:
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(np.array(r))

                            # Estimate plane and compute normal
                            if len(point_cloud.points) > 3:
                                # Compute plane using Open3D's estimate_normals
                                point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                                
                                # Use PCA or RANSAC to fit a plane to the point cloud
                                plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
                                [a, b, c, d] = plane_model
                                
                                # The normal vector of the plane is (a, b, c)
                                normal_vector = np.array([-a, -b, -c])
                                normal_vector = normal_vector / np.linalg.norm(normal_vector)  # Normalize the vector
                                origin = np.mean(np.array(r), axis=0)  # Use the mean of the points as the origin

                                # publish_point_cloud(point_cloud, label)

                                for point in random_points:
                                    point_2d = (int(point[0]), int(point[1]))  # 정수로 변환 후 사용
                                    cv.circle(image, point_2d, 5, (255, 0, 0), -1)  # 빨간색으로 특징점을 그립니다.

                                for point in image_points:
                                    point_2d = tuple(point.astype(int))
                                    cv.circle(image, point_2d, 5, (0, 255, 0), -1)  # 초록색으로 특징점을 그립니다.
                                
                                position_2d = image_points[0].astype(int)
                                position_3d = get_coordinate_depth(depth_image, position_2d)
                                x_value = position_3d[0]
                                y_value = position_3d[1]
                                z_value = position_3d[2]  # Extract z value

                                label_index = int(names[idx])  # Get the integer index from names
                                if label_index < len(name_list):  # Ensure index is valid
                                    label = name_list[label_index]
                                    if label in label_dict and image_points.shape[0] == 4:
                                        success, rotation_vector, translation_vector = cv.solvePnP(
                                            label_dict[label], image_points, camera_matrix, dist_coeffs
                                        )
                                    Rotm, _ = cv.Rodrigues(rotation_vector)
                                    corrected_rotation_vector, _ = cv.Rodrigues(align_z_axis_and_get_new_rotation_matrix(normal_vector, Rotm))
                                
                                marker_array_msg_red_robot = MarkerArray()
                                corrected_marker_array_msg_red_robot = MarkerArray()
                                axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]])
                                imgpts, _ = cv.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

                                # Draw the axes on the frame
                                image = cv.drawFrameAxes(image, camera_matrix, dist_coeffs, rotation_vector, translation_vector, length=2)
                                face_center_2d = np.mean(image_points, axis=0).astype(int)
                                face_center_3d = get_coordinate_depth(depth_image, face_center_2d)
                                face_center_3d = face_center_3d * 0.01
                                width, height, depth = 0.065, 0.065, 0.065
                                tv = np.array([translation_vector[0][0], translation_vector[1][0], translation_vector[2][0]], dtype=np.float32) * 0.01
                                vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, rotation_vector, width, height, depth)
                                vertices_PnP = get_cuboid_vertices_from_face_center(tv, rotation_vector, width, height, depth)

                                corrected_vertices_measured = get_cuboid_vertices_from_face_center(face_center_3d, corrected_rotation_vector, width, height, depth)
                                corrected_vertices_PnP = get_cuboid_vertices_from_face_center(tv, corrected_rotation_vector, width, height, depth)

                                publish_cuboid_marker(vertices_measured, 4, marker_array_msg_red_robot, 4)
                                publish_cuboid_marker(corrected_vertices_measured, 7, corrected_marker_array_msg_red_robot, 7)
                            
                                for _ in range(len(names)):
                                    # Clear previous markers and publish new markers
                                    delete_marker = Marker()
                                    delete_marker.action = Marker.DELETEALL
                                    corrected_marker_array_msg_red_robot.markers.insert(0, delete_marker)
                                    marker_array_msg_red_robot.markers.insert(0, delete_marker)

                                if len(names) != 0:
                                    detection_msg = MarkerArray()
                                    detection_msg.markers = marker_array_msg_red_robot.markers

                                # Update the objects dictionary
                                if label in objects:
                                    objects[label]['label'].append(label)
                                    objects[label]['rvec'].append(rotation_vector)
                                    objects[label]['position'].append(position_3d)
                                    objects[label]['vertices'].append(corrected_vertices_measured)
                                    objects[label]['keypoints_2d'].append(image_points)
                                    objects[label]['face_center_3d'].append(face_center_3d)
                                    objects[label]['corrected_rotation_vector'].append(corrected_rotation_vector)
                                    objects[label]['normal_vector'].append(normal_vector)
                                    objects[label]['area'].append(area)




        image_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)


        try:
            Image_publisher.publish(bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8"))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")


        try:
            # [기존의 코드: 이미지 처리 및 objects 업데이트 부분]

            # 프레임마다 objects 업데이트 후 병합
            for label in objects:
                for key in objects[label]:
                    combined_objects[label][key].extend(objects[label][key])
            
            
            frame_count += 1
            # print(f"frame_count : {frame_count}")
            if frame_count >= 5:
                pc_np = np.array(list(realsense_pointcloud))
                cloud_from_camera = o3d.geometry.PointCloud()
                cloud_from_camera.points = o3d.utility.Vector3dVector(pc_np)
                combined_objects = objects_to_robot_coordinate(combined_objects)
                robot_coordinate_cloud = pointcloud_to_robot_coordinate(cloud_from_camera)
                # print("5 frames processed. Stopping subscription.")

                def remove_floor_from_point_cloud(point_cloud):
                    """
                    z 좌표가 0인 포인트들을 필터링하여 바닥을 제거합니다.
                    
                    Args:
                    - point_cloud: Open3D 포인트 클라우드 객체 (o3d.geometry.PointCloud)
                    
                    Returns:
                    - 바닥이 제거된 새로운 포인트 클라우드 객체
                    """
                    # numpy 배열로 변환
                    points = np.asarray(point_cloud.points)

                    # z 값이 0인 포인트들을 필터링하여 제거
                    filtered_points = points[points[:, 2] > 0.015]

                    # 새로운 포인트 클라우드 객체 생성
                    filtered_point_cloud = o3d.geometry.PointCloud()
                    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

                    return filtered_point_cloud
                def save_transformed_cloud(angle_count, robot_coordinate_cloud, voxel_size=0.005):
                    # Print the number of points before downsampling
                    num_points_before = len(robot_coordinate_cloud.points)
                    # print(f"Number of points before downsampling: {num_points_before}")
                    
                    # Apply voxel downsampling
                    downsampled_cloud = robot_coordinate_cloud.voxel_down_sample(voxel_size)
                    downsampled_cloud = remove_floor_from_point_cloud(downsampled_cloud)
                    
                    # Print the number of points after downsampling
                    num_points_after = len(downsampled_cloud.points)
                    # print(f"Number of points after downsampling: {num_points_after}")
                    
                    # Save the downsampled point cloud
                    file_name = f"PointCloudFromCircleRobot/transformed_pointcloud_view_{angle_count+1}.ply"
                    o3d.io.write_point_cloud(file_name, downsampled_cloud)
                    # print(f"Point cloud saved to {file_name}")

                def save_cloud(angle_count, robot_coordinate_cloud, voxel_size=0.005):
                    # Print the number of points before downsampling
                    num_points_before = len(robot_coordinate_cloud.points)
                    # print(f"Number of points before downsampling: {num_points_before}")
                    
                    # Apply voxel downsampling
                    downsampled_cloud = robot_coordinate_cloud.voxel_down_sample(voxel_size)
                    
                    # Print the number of points after downsampling
                    num_points_after = len(downsampled_cloud.points)
                    # print(f"Number of points after downsampling: {num_points_after}")
                    
                    # Save the downsampled point cloud
                    file_name = f"PointCloudFromCircleRobot/pointcloud_view_{angle_count+1}.ply"
                    o3d.io.write_point_cloud(file_name, downsampled_cloud)
                    # print(f"Point cloud saved to {file_name}")


                if angle_count == 0:
                    combined_objects_1 = combined_objects
                    cloud_from_view_1 = robot_coordinate_cloud
                    save_transformed_cloud(angle_count, cloud_from_view_1)
                    save_cloud(angle_count, cloud_from_camera)
                elif angle_count == 1:
                    combined_objects_2 = combined_objects
                    cloud_from_view_2 = robot_coordinate_cloud
                    save_transformed_cloud(angle_count, cloud_from_view_2)
                    save_cloud(angle_count, cloud_from_camera)
                elif angle_count == 2:
                    combined_objects_3 = combined_objects
                    cloud_from_view_3 = robot_coordinate_cloud
                    save_transformed_cloud(angle_count, cloud_from_view_3)
                    save_cloud(angle_count, cloud_from_camera)
                elif angle_count == 3:
                    combined_objects_4 = combined_objects
                    cloud_from_view_4 = robot_coordinate_cloud
                    save_transformed_cloud(angle_count, cloud_from_view_4)
                    save_cloud(angle_count, cloud_from_camera)  

                color_image_subscriber.unregister()
                pointcloud_sub.unregister()
                depth_image_sub.unregister()
                camera_info_sub.unregister()


                frame_count = 0  # 카운터 초기화
                flag = False
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")


    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def see():
    # 여기서 yolo로 딕셔너리 다 담기, 위치별 N회씩
    # Pointcloud 이미지는 1회만
    global color_image_subscriber, pointcloud_sub, depth_image_sub, camera_info_sub, Image_publisher
    global point_cloud_pub_color_logo, point_cloud_pub_black_logo, point_cloud_pub_round_logo, point_cloud_pub_blue_robot, point_cloud_pub_red_robot
    global point_cloud_pub_color_logo2, point_cloud_pub_black_logo2, point_cloud_pub_round_logo2, point_cloud_pub_blue_robot2, point_cloud_pub_red_robot2
    global multi_cuboid_pub
    global flag
    flag = True
    frame_count = 0
    # Subscribers
    color_image_subscriber = color_image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, rgb_msg_callback)
    pointcloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback)
    depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_msg_callback)
    camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, color_camera_info_callback)
    
    #Publishers
    Image_publisher = rospy.Publisher('yolo_image', Image, queue_size=10)
    point_cloud_pub_color_logo = rospy.Publisher('/color_logo_cuboid', PointCloud2, queue_size=10)
    point_cloud_pub_black_logo = rospy.Publisher('/black_logo_cuboid', PointCloud2, queue_size=10)
    point_cloud_pub_round_logo = rospy.Publisher('/round_logo_cuboid', PointCloud2, queue_size=10)
    point_cloud_pub_blue_robot = rospy.Publisher('/blue_robot_cuboid', PointCloud2, queue_size=10)
    point_cloud_pub_red_robot = rospy.Publisher('/red_robot_cuboid', PointCloud2, queue_size=10)
    
    point_cloud_pub_color_logo2 = rospy.Publisher('/color_logo_cuboid_no_icp', PointCloud2, queue_size=10)
    point_cloud_pub_black_logo2 = rospy.Publisher('/black_logo_cuboid_no_icp', PointCloud2, queue_size=10)
    point_cloud_pub_round_logo2 = rospy.Publisher('/round_logo_cuboid_no_icp', PointCloud2, queue_size=10)
    point_cloud_pub_blue_robot2 = rospy.Publisher('/blue_robot_cuboid_no_icp', PointCloud2, queue_size=10)
    point_cloud_pub_red_robot2 = rospy.Publisher('/red_robot_cuboid_no_icp', PointCloud2, queue_size=10)
    
    multi_cuboid_pub = rospy.Publisher('/multiple_cuboids', MarkerArray, queue_size=10)

def handle_detection_service(request):
    multi_view_client = rospy.ServiceProxy('/multi_view_poses', multi_viewing_pose)
    multi_view_poses = multi_view_client()
    # print(f"view 1 : {multi_view_poses.view1}")
    # print(f"view 2 : {multi_view_poses.view2}")
    # print(f"view 3 : {multi_view_poses.view3}")
    # print(f"view 4 : {multi_view_poses.view4}")
    multi_view_pose = [multi_view_poses.view1, multi_view_poses.view2, multi_view_poses.view3, multi_view_poses.view4]
    response = vision_verticesResponse()
    global angle_count
    rate = rospy.Rate(0.5)  # 0.5 Hz for a 2-second interval
    object_position = [multi_view_poses.view1[0], multi_view_poses.view1[1], multi_view_poses.view1[2]]
    height = 400
    radius = 200
    
    vel = [0.5, 0.5]  # Velocity
    acc = [0.5, 0.5]  # Acceleration
    time_ = 4  # Time duration for each movement

    angle_count = 0
    # angle_step = 120
    # print(f"Starting from Scene 1")
    for i, pose in enumerate(multi_view_pose):
        pose = list(pose)
        print(f"pose {i} : {pose}")
        movejx(pose, vel =20, acc = 10, time =3)
        print(f"collecting scene {i + 1}")
        see()
        while flag == True :
            rospy.sleep(0.1)  # Prevent CPU overuse by sleeping
        print(f"scene {i+1} succesfully collected")
        if (i+2)<5:
            print(f"Moving On To Scene {i+2}")
        else :
            print("**Succesfulley collected all scenes**")
        rate.sleep()
        angle_count = angle_count + 1
        
    moveline((object_position[0],object_position[1],height,0,180,0),vel,acc,time_) # pick place 중앙으로
    try :
        get_current_rotm_posj = rospy.ServiceProxy('/dsr01m1013/aux_control/get_current_posj', GetCurrentPosj)

        get_current_posj_response = get_current_rotm_posj()
        # print(f"get_current_posj_response : {get_current_posj_response}")
        pos = list(get_current_posj_response.pos[:])
        pos[5] = -180
        # print(pos)

        movej(pos, 0.5, 0.5, 2)
        # print("safely moved joint6 to 0 deg")
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

    try :
        get_current_rotm_posj = rospy.ServiceProxy('/dsr01m1013/aux_control/get_current_posj', GetCurrentPosj)

        get_current_posj_response = get_current_rotm_posj()
        # print(f"get_current_posj_response : {get_current_posj_response}")
        pos = list(get_current_posj_response.pos[:])
        pos[5] = 0
        # print(pos)

        movej(pos, 0.5, 0.5, 2)
        # print("safely moved joint6 to 0 deg")
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    # print(f"Constructing Map from collected Scenes.. \n")
    # combined_objects_list = [combined_objects_1, combined_objects_2, combined_objects_3,combined_objects_4 ]
    
    combined_objects_list = []

    # Try to add each object to the list if it's defined
    try:
        combined_objects_list.append(combined_objects_1)
    except NameError:
        pass

    try:
        combined_objects_list.append(combined_objects_2)
    except NameError:
        pass

    try:
        combined_objects_list.append(combined_objects_3)
    except NameError:
        pass

    try:
        combined_objects_list.append(combined_objects_4)
    except NameError:
        pass


    final_combined_objects = merge_dicts(combined_objects_list)
    # print("merged")
    # print_objects_numbers(final_combined_objects)
    final_points, final_normals ,final_rvecs,obj_names=process_combined_objects(final_combined_objects)
    # for name, final_point in (zip(obj_names, final_points)):
        # print(f"{name} :")
        # print(f"final_points :{final_point}")
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(np.array(final_points))
    vertices_before_icp, icp_vertices, names = publish_multiple_cuboids(obj_names, final_points, final_rvecs) # 이미 좌표계 변환된 결과가 인풋이다!
    
    for vertices, name in zip(icp_vertices, names) :
        vertices = np.array(vertices)
        flat_vertices = vertices.flatten().tolist()
        response.vertices.extend(flat_vertices)
        response.names.append(name)
        # print(f"1 : {vertices}")
        # vertices = vertices[np.lexsort((vertices[:, 2], vertices[:, 1], vertices[:, 0]))]
        # # print(f"2 : {vertices}")
        # vertices = [list(vertices[1]), list(vertices[5]), list(vertices[7]), list(vertices[3]),
        #             list(vertices[0]), list(vertices[4]), list(vertices[6]), list(vertices[2])]

        # 콤마 없이 출력하기 위해 numpy 배열로 변환한 뒤 다시 리스트로 변환
        print(f"{name}")
        for i, vertex in enumerate(vertices):
            print(f"Vertices{i} : {vertex} ")
        print()
    print('done')

    return response

def main():
    global rgb_image_sub, depth_image_sub, camera_info_sub, pointcloud_sub, flag, angle_count
    global combined_objects_1, combined_objects_2, combined_objects_3, combined_objects_4
    global cloud_from_view_1, cloud_from_view_2, cloud_from_view_3, cloud_from_view_4
    global detection_results_pub

    rospy.init_node('object_detection_node')
    service = rospy.Service('/collision_detect/vision_vertices', vision_vertices, handle_detection_service)
    rospy.spin()

if __name__ == "__main__":
    main()
