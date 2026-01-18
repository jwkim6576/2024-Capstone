# General system modules
import rospy
import numpy as np
import shutil

# Matplotlib for plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# SciPy for spatial transformations
from scipy.spatial.transform import Rotation as R

# ROS service imports for palletizing system
from palletizing_system.srv import multi_viewing_pose, multi_viewing_poseResponse

# ROS service imports for Doosan robot
from dsr_msgs.srv import (
    MoveJointx, MoveJointxRequest,
    MoveJoint, MoveJointRequest,
    MoveLine, MoveLineRequest,
    GetCurrentPose, GetCurrentPoseRequest
)

# -------------------------------------------------------두산로봇 함수 ----------------------------------------------------------------------------------------

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

def moveline(posx, vel, acc, time):
    rospy.wait_for_service('/dsr01m1013/motion/move_line')
    try:
        move_line = rospy.ServiceProxy('/dsr01m1013/motion/move_line', MoveLine)
        req = MoveLineRequest()
        req.pos = posx
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = 0
        req.mode = 0
        req.blendType = 0
        req.syncType = 0

        response = move_line(req)
        
        # rospy.loginfo("MoveLine Success: %f %f %f %f %f %f", *posx)
    except rospy.ServiceException as e:
        #rospy.logerr("Failed to call service move_line: %s", e)
        rospy.signal_shutdown("Service call failed")

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

# ---------------------------------------------------------------------------------------------------------------------------------------------------------

# ----------------------------------------------------다른 클라이언트 관련 함수-----------------------------------------------------------------------------------------

# EMPTY

# ---------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------플롯 관련 함수-----------------------------------------------------------------------------------------
def plot_box(ax, vertices, color='cyan'):
    # Define the six faces of the box using the vertices
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]], # Bottom face
        [vertices[4], vertices[5], vertices[6], vertices[7]], # Top face
        [vertices[0], vertices[1], vertices[5], vertices[4]], # Front face
        [vertices[2], vertices[3], vertices[7], vertices[6]], # Back face
        [vertices[1], vertices[2], vertices[6], vertices[5]], # Right face
        [vertices[4], vertices[7], vertices[3], vertices[0]]  # Left face
    ]
    
    # Create a 3D polygon collection and add it to the plot
    box = Poly3DCollection(faces, linewidths=1, edgecolors='r')
    box.set_facecolor(color)
    ax.add_collection3d(box)

def plot_plane(ax, p1, p2, p3, p4):
    # 주어진 점들을 NumPy 배열로 정의합니다.
    p1, p2, p3, p4 = map(np.array, [p1, p2, p3, p4])
    
    # 네 점을 이은 다각형 평면을 정의합니다.
    verts = [[p1, p2, p3, p4]]
    poly = Poly3DCollection(verts, color="cyan", alpha=0.5)
    ax.add_collection3d(poly)

    # 네 점 플롯
    # ax.scatter(*p1, color="red", s=50, label="Point 1")
    # ax.scatter(*p2, color="green", s=50, label="Point 2")
    # ax.scatter(*p3, color="blue", s=50, label="Point 3")
    # ax.scatter(*p4, color="purple", s=50, label="Point 4")
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

def plot_line(ax, p1, p2, label=None, color = 'black'):
    # 선분을 그리는 함수 (선의 굵기를 3으로 설정)
    line_x = [p1[0], p2[0]]
    line_y = [p1[1], p2[1]]
    line_z = [p1[2], p2[2]]
    
    # label이 있으면 적용, 없으면 label 매개변수를 생략
    if label is not None:
        ax.plot(line_x, line_y, line_z, color = color, linewidth=3, label=label)
        ax.legend()
    else:
        ax.plot(line_x, line_y, line_z, color = color, linewidth=3)  # label 없이 선만 그리기

# ------------------------------------------------------------------------------------------------------------------------------------------------------------

# -------------------------------------------------------------------좌표계 관련 함수------------------------------------------------------------------------------
def rotate_point_around_axis(p1, p2, point, theta):
    """
    주어진 축(p1, p2)과 각도(theta)만큼 임의의 점(point)을 회전시킵니다.
    
    Parameters:
    - p1, p2: 회전 축의 두 점 (numpy array 또는 list)
    - point: 회전시킬 점 (numpy array 또는 list)
    - theta: 회전 각도 (라디안)
    
    Returns:
    - rotated_point: 회전된 점의 좌표 (numpy array)
    """
    # p1, p2, point를 numpy 배열로 변환
    p1 = np.array(p1)
    p2 = np.array(p2)
    point = np.array(point)
    
    # 회전축 벡터 정의 (단위 벡터로 정규화)
    axis = p2 - p1
    axis = axis / np.linalg.norm(axis)
    
    # 회전 변환 정의
    rotation = R.from_rotvec(theta * axis)
    
    # 회전 적용 (축의 기준점을 원점으로 이동한 후 회전하고 다시 원래 위치로 이동)
    rotated_point = rotation.apply(point - p1) + p1
    
    return rotated_point

def get_rpy(R): # R = np.array([xvec,yvec,zvec]).T
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

# 평행이동 된 posx와 그리퍼의 체적(상태)을 받아 그리퍼의 체적을 선으로 계산 volume은 np.array(--> 기준점 평행이동[x,y,z].[x,y,z]<-- 체적) 모두 양수로 계산
def posx2line(posx, volume):
    # 먼저 평행이동된 점 posx_p를 구하자
    unitVec = rpy2vector(posx[3],posx[4],posx[5])
    
    posx_p = posx_calculate(posx, volume[0][0], volume[0][1], volume[0][2])
    posx_xyz = np.array([posx_p[0], posx_p[1], posx_p[2]])

    

    # gripper_volume 은 ([[x+,y+,z+],[x+,y+,z-], ...]) 1분면부터 4분면 까지
    # gripper_vector 는 ([xx, xy, xz],[yx, yy, yz],[zx,zy,zz])
    gripper_vector = np.array([unitVec[0] * volume[1][0],unitVec[1] * volume[1][1],unitVec[2] * volume[1][2]])

    gripper_volume = np.array([[posx_xyz + gripper_vector[0] + gripper_vector[1] + gripper_vector[2],
                                posx_xyz + gripper_vector[0] + gripper_vector[1]],    #1사분면 라인

                                [posx_xyz - gripper_vector[0] + gripper_vector[1] + gripper_vector[2],
                                posx_xyz - gripper_vector[0] + gripper_vector[1]],    #2사분면 라인
                                
                                [posx_xyz - gripper_vector[0] - gripper_vector[1] + gripper_vector[2],
                                posx_xyz - gripper_vector[0] - gripper_vector[1]],    #3사분면 라인

                                [posx_xyz + gripper_vector[0] - gripper_vector[1] + gripper_vector[2],
                                posx_xyz + gripper_vector[0] - gripper_vector[1]]])   #4사분면 라인

    return gripper_volume

# ------------------------------------------------------------------------------------------------------------------------------------------------------------

# ---------------------------------------------------------------------장애물 검사 관련 함수---------------------------------------------------------------------------

def surface_points_divide(surface, points):
    #surface가 points를 나누는지(divide) 검사하는 함수
    #나누면 True 안나누면 False return
    #surface는 np array 4개의 점 points는 n개의 점

    #surface에서 임의의 세개의 점들을 뽑아 외적을 통해 normal 벡터를 구한다 크기는 규격화 x   + surface를 이루는 모든 line을 정의한다

    v1 = surface[0]-surface[1]
    v2 = surface[1]-surface[2]
    v3 = surface[2]-surface[3]
    v4 = surface[3]-surface[0]


    normal_vec = np.cross(v1,v2) #[x1,y1,z1] 평면 방정식은 normal_vec[0]x + normal_vec[1]y + normal_vec[2]z + d -= 0;
    d = -np.dot(normal_vec,surface[0])

    # 점 하나하나 검사

    # 양수면 True 음수면 False인 bool_list
    bool_list = []


    for i in range(points.shape[0]):
        # 점을 평면에 투영

        t = -(np.dot(normal_vec,points[i])+d)/np.dot(normal_vec,normal_vec)
        x = normal_vec[0]*t + points[i][0]
        y = normal_vec[1]*t + points[i][1]
        z = normal_vec[2]*t + points[i][2]

        point = np.array([x,y,z]) # point: points가 surface에 투영되는 점

        point_surface_vector= np.array([[]])

        for j in range(surface.shape[0]):
            point_surface_vector = np.append(point_surface_vector, point - surface[j])     # point가 끝점, surface가 시작점인 벡터 4개
        point_surface_vector = point_surface_vector.reshape(4,3)

        space_in = 'out'
        
        radian = 0

        for j in range(point_surface_vector.shape[0]):
            cos_theta = np.dot(point_surface_vector[j-1], point_surface_vector[j]) / \
                        (np.linalg.norm(point_surface_vector[j-1]) * np.linalg.norm(point_surface_vector[j]))
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            radian += np.arccos(cos_theta)
            
        if (radian >=2*3.14):
            space_in = 'in'
        
        if(space_in == 'in'):
            # 평면의 정사영 안에 있는 점들 뽑음. points무리를 surface가 나누는지 검사

            # ax.scatter(points[i][0], points[i][1], points[i][2], color="orange", s=20)

            initial_value = np.dot(normal_vec,points[i]) + d

            if(initial_value >= 0):
                bool_list.append(True)

            else:
                bool_list.append(False)

    # all true 인가?
    if(all(bool_list)):
        return False
    # all False 인가?
    if (all(not value for value in bool_list)):
        return False

    else:
        return True

def obstacle_volume(obstacle, volume):
    # volume이 obstalce을 등분하는가 등분하면 True 등분 안하면 False 반환
    # 장애물 (여러개의 면) np.array (n,4,3)
    # 볼륨,그리퍼 (4개의 선) np.array (4,2,3), posx2line 순서
    # volume이 obstacle을 검사하고 obstacle도 volume을 검사해야 한다.

    # 볼륨을 면으로 만들어 각각의 면을 검사하자

    terminal_width = shutil.get_terminal_size().columns

    surfaces = vertices2surface(volume.reshape(8,3))



    surface_inspect_obstacle = False
    for i in range(surfaces.shape[0]): # 한개의 서피스에 대해 검사
        for j in range(obstacle.shape[0]): # 한개의 obstacle 면에 대해 검사
            divide = surface_points_divide(surfaces[i], obstacle[j])
            if(divide):
                surface_inspect_obstacle = True
                break
        if(surface_inspect_obstacle):
            break


    obstalce_inspect_surface = False
    for i in range(obstacle.shape[0]): # 한개의 obstacle 면이 volume 검사
        for j in range(surfaces.shape[0]) :
            divide = surface_points_divide(obstacle[i], surfaces[j])
            if(divide):
                obstalce_inspect_surface = True
                break

        if(obstalce_inspect_surface):
            break

    if(surface_inspect_obstacle or obstalce_inspect_surface):
        return True

    else:
        return False

def vertices2surface(vertices):
    # vertices = np.araay (8,3) 을 받으면 (posx2line을 써서 나온 값) 6개의 면을 가진 surface = np.array(6,4,3)을 반환
    surface = np.array([
    [vertices[1], vertices[3], vertices[5], vertices[7]], # Bottom face
    [vertices[0], vertices[2], vertices[4], vertices[6]], # Top face
    [vertices[7], vertices[6], vertices[4], vertices[5]], # Front face
    [vertices[1], vertices[0], vertices[2], vertices[3]], # Back face
    [vertices[1], vertices[0], vertices[6], vertices[7]], # Right face
    [vertices[3], vertices[2], vertices[4], vertices[5]]  # Left face
        ])

    return surface

def findViewpoint(vertex,vertex_o, highMiddlepoint0, highMiddlepoint1, zlineVector, highSurface_center, lowSurface_center):

    viewPoint1 = (highMiddlepoint0 + highMiddlepoint1) /2

    viewPoint2 = highSurface_center.copy()

    flag1 = False
    flag2 = False

    while(1):
        if (not flag1):
            viewVector0 = viewPoint2-highMiddlepoint0
            viewVector1 = viewPoint2-highMiddlepoint1
            dotValue = np.dot(viewVector0, viewVector1)/np.linalg.norm(viewVector0)/np.linalg.norm(viewVector1)
            theta0 = np.arccos(dotValue) * 180 / np.pi

            viewVector2 = viewPoint1 - viewPoint2
            viewVector3 = vertex - viewPoint2
            dotValue = np.dot(viewVector2, viewVector3)/np.linalg.norm(viewVector2)/np.linalg.norm(viewVector3)
            theta1 = np.arccos(dotValue) * 180 / np.pi
            
            

            if (theta0 > 60 or theta1 > 15):
                viewPoint2 += zlineVector * 5 # 5mm 단위 계산
            else:
                flag1 = True
                

        
        if (flag1):
            zvec = viewPoint1 - viewPoint2
            zunit = zvec / np.linalg.norm(zvec)
            
            xvec = highMiddlepoint0 - highMiddlepoint1
            xunit = xvec/ np.linalg.norm(xvec)

            yunit = np.cross(zunit, xunit)

            zunitVec = np.array([0.0,0.0,1.0])

            if(np.dot(yunit,zunitVec) > 0):
                xunit = -xunit
                yunit = -yunit

            R = np.array([xunit, yunit, zunit]).T
            y,p,r = get_rpy(R)

            posx = np.array([viewPoint2[0],viewPoint2[1],viewPoint2[2],y,p,r])
            posx = posx_calculate(posx, -cam[0],-cam[1],-cam[2])

            if(posx[2]<highSurface_center[2]+10):
                viewPoint2 += zlineVector * 5
            else :
                flag2 = True
                
        if(flag1 and flag2):
            vector = vertex_o - vertex
            vector /= np.linalg.norm(vector)
            viewVector4 = lowSurface_center - viewPoint2
            viewVector5 = viewPoint1 - viewPoint2
            dotValue = np.dot(viewVector4, viewVector5)/np.linalg.norm(viewVector4)/np.linalg.norm(viewVector5)
            theta2 = np.arccos(dotValue) * 180 / np.pi
            if(theta2 > 17.5):
                print('theta2 =', theta2)
                viewPoint2[0] += vector[0] * 1
                viewPoint2[1] += vector[1] * 1 
                viewPoint2[2] += vector[2] * 1
            else:
                
                zvec = viewPoint1 - viewPoint2
                zunit = zvec / np.linalg.norm(zvec)
                
                xvec = highMiddlepoint0 - highMiddlepoint1
                xunit = xvec/ np.linalg.norm(xvec)

                yunit = np.cross(zunit, xunit)

                zunitVec = np.array([0.0,0.0,1.0])

                if(np.dot(yunit,zunitVec) < 0):
                    xunit = -xunit
                    yunit = -yunit

                R = np.array([xunit, yunit, zunit]).T
                y,p,r = get_rpy(R)

                posx = np.array([viewPoint2[0],viewPoint2[1],viewPoint2[2],y,p,r])
                posx = posx_calculate(posx, -cam[0],-cam[1],-cam[2])

                break


    return posx

def findBoxposx(vertices, cam):
    # 박스를 보기 위한 자세를 만드는 함수
    # vertices 는 8개의 점 (8,3) vertices 0, 1, 2, 3은 무조껀 윗면의 꼭짓점으로 생각 
    # 밑에 4개의 변과 위에 4개의 변의 상호작용을 알아야 한다
    # 한 꼭짓점에 대해서 볼때 꼭짓점을 성분으로 하는 두개의 직선과 대각선의 직선이 무조건 만나게 되어 있다



    posx = np.array([])

    # 먼저 surface_center와 volume_center를구하자
    volume_center = np.array([0.0,0.0,0.0])

    for vertex in vertices:
        volume_center += vertex
    volume_center /= 8


    highSurface_center = (vertices[0]+vertices[1]+vertices[2]+vertices[3])/4
    lowSurface_center = (vertices[4]+vertices[5]+vertices[6]+vertices[7])/4


    zlineVector = highSurface_center-volume_center
    zlineVector = zlineVector/np.linalg.norm(zlineVector)





    # vertices 0을 본다고 생각하자 그러면 1,3 점과의 중점

    highMiddlepoint0 = (vertices[0] + vertices[1])/2
    highMiddlepoint1 = (vertices[0] + vertices[3])/2

    posx0 = findViewpoint(vertices[0],vertices[2],highMiddlepoint0, highMiddlepoint1, zlineVector, highSurface_center,lowSurface_center)


    # vertices 1 을 볼려면

    highMiddlepoint0 = (vertices[1] + vertices[0])/2
    highMiddlepoint1 = (vertices[1] + vertices[2])/2

    posx1 = findViewpoint(vertices[1],vertices[3],highMiddlepoint0, highMiddlepoint1, zlineVector, highSurface_center,lowSurface_center)

    # 2
    highMiddlepoint0 = (vertices[2] + vertices[1])/2
    highMiddlepoint1 = (vertices[2] + vertices[3])/2

    posx2 = findViewpoint(vertices[2],vertices[0],highMiddlepoint0, highMiddlepoint1, zlineVector, highSurface_center,lowSurface_center)
    

    #3
    highMiddlepoint0 = (vertices[3] + vertices[0])/2
    highMiddlepoint1 = (vertices[3] + vertices[2])/2

    posx3 = findViewpoint(vertices[3],vertices[1],highMiddlepoint0, highMiddlepoint1, zlineVector, highSurface_center,lowSurface_center)
        

    posx = np.array([posx0, posx1 , posx2, posx3])


    # posx가 장애물에 부딪히면 z값을 더 올린다.


    return posx

def handle_service(request):
    global cam
    response = multi_viewing_poseResponse()
    vertices = np.array([

    # [350, 200, 100],  # V0
    # [650, 200, 100],  # V1
    # [650, 800, 100],  # V2
    # [350, 800, 100],  # V3
    # [350, 200, 0],  # V0
    # [650, 200, 0],  # V1
    # [650, 800, 0],  # V2
    # [350, 800, 0],  # V3
    
    # # # 오른쪽 위
    # [-126.95901, 378.20737, 100],
    # [471.26535, 380.82208, 100],
    # [468.45398, 677.81250, 100],
    # [-129.48758, 670.38696, 100],
    # [-126.95901, 378.20737, 0],
    # [471.26535, 380.82208, 0],
    # [468.45398, 677.81250, 0],
    # [-129.48758, 670.38696, 0]
    
    
    # 오른쪽 아래
    [-127.32172, 550.04443, 100],
    [468.93512, 560.19818, 100],
    [464.13327, 854.77448, 100],
    [-130.28603, 849.80872, 100],
    [-127.32172, 550.04443, 0],
    [468.93512, 560.19818, 0],
    [464.13327, 854.77448, 0],
    [-130.28603, 849.80872, 0]

    
    
    ])

    cam = np.array([0 ,-110, -(210)])
    posx = findBoxposx(vertices, cam)
    
    for i in range(4):
        posx[i][2] -= 150  # z값은 리스트의 세 번째 요소이므로 [2]에 접근하여 100 감소
    
    response.view1 = posx[0]
    response.view2 = posx[1]
    response.view3 = posx[2]
    response.view4 = posx[3]

    return response

def main():
    rospy.init_node('multi_view_pose_generator')

    service = rospy.Service('/multi_view_poses', multi_viewing_pose, handle_service)

    rospy.spin()

if __name__ == "__main__":
    main()
