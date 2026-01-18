# General system modules
import rospy
import numpy as np
import os
import shutil

# Matplotlib for plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# SciPy for spatial transformations
from scipy.spatial.transform import Rotation as R

# ROS message imports
from palletizing_system.msg import servoing_plot, pick_plot

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

def plot_plane(ax, p1, p2, p3, p4, color = 'cyan'):
    # 주어진 점들을 NumPy 배열로 정의합니다.
    p1, p2, p3, p4 = map(np.array, [p1, p2, p3, p4])
    
    # 네 점을 이은 다각형 평면을 정의합니다.
    verts = [[p1, p2, p3, p4]]
    poly = Poly3DCollection(verts, color=color, alpha=0.5)
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

def surface2vertices(surface):
    # 6개의 면을 가진 surface = np.array(6,4,3)을 받아 vertices = np.araay (8,3) 반환
    
    vertices = np.array([surface[1][0],surface[0][0],surface[1][1],surface[0][1],surface[1][2],surface[0][2],surface[1][3],surface[0][3]])

    return vertices

def plot_servoing(ax, obstacle, vertices, volumes, posx, servoing_point, robot):

    print("obstacle = ", obstacle)
    print("vertices = ", vertices)
    print("volumes = ", volumes)
    print("servoing_posx =", posx)
    print("servoing_point=", servoing_point)

    # obstacle 출력 
    for i in range(obstacle.shape[0]):
        if(i ==4 or i ==5):
            continue
        plot_plane(ax, obstacle[i][0], obstacle[i][1], obstacle[i][2], obstacle[i][3])

    # vertices 출력
    for i in range(vertices.shape[0]):
        plot_box(ax, vertices[i], color = 'yellow')

    # servoing_point 출력
    for i in range(servoing_point.shape[0]):
        ax.scatter(*servoing_point[i], color="blue", s = 50)

    # 최종 servoing_posx에 해당하는 manipulator 플롯
    for i in range(posx.shape[0]):
        # ax.scatter(servoing_posx[i][0],servoing_posx[i][1],servoing_posx[i][2],color="red", s=40)
        for j in range(volumes.shape[0]):
            servoing_volume = posx2line(posx[i],volumes[j])
            for k in range(servoing_volume.shape[0]):
                if(j ==0):
                    plot_line(ax, servoing_volume[k][0], servoing_volume[k][1], label="gripper", color = 'blue')
                elif(j == 1):
                    plot_line(ax, servoing_volume[k][0], servoing_volume[k][1], label='camera', color = 'red')
                elif(j==2):
                    plot_line(ax, servoing_volume[k][0], servoing_volume[k][1], label='manipulator', color = 'green')
                elif(j==3):
                    plot_line(ax, servoing_volume[k][0], servoing_volume[k][1], label='sight', color = 'purple')
    # for i in range(robot.shape[0]):
    #     plot_line(ax, robot[i][0], robot[i][1], label=None, color = 'red')

def servoing_plotCallback(data):

    global plot_, obstacleG, verticesG, volumesG, posxG, servoing_pointG, robotG

    obstacleG = np.array(data.obstacle)
    obstacleG = obstacleG.reshape(-1,4,3)

    verticesG = np.array(data.vertices)
    verticesG = verticesG.reshape(-1,8,3)

    volumesG = np.array(data.volumes)
    volumesG = volumesG.reshape(-1,2,3)

    posxG = np.array(data.posx)
    posxG = posxG.reshape(-1,6)

    servoing_pointG = np.array(data.servoing_point)
    servoing_pointG = servoing_pointG.reshape(-1,3)
    
    robotG = np.array(data.robot)
    robotG = robotG.reshape(4,2,3)
    
    plot_ = 'servoing'

def plot_pick(ax, obstacle, vertices, volumes, posx, robot):
    print("obstacle = ", obstacle)
    print("vertices = ", vertices)
    print("volumes = ", volumes)
    print("pick_posx =", posx)


    # obstacle 출력 
    for i in range(obstacle.shape[0]):
        if(i ==4 or i ==5):
            continue
        plot_plane(ax, obstacle[i][0], obstacle[i][1], obstacle[i][2], obstacle[i][3])

    # vertices 출력
    for i in range(vertices.shape[0]):
        plot_box(ax, vertices[i], color = 'yellow')

    # 최종 final_posx에 해당하는 manipulator 플롯
    for i in range(posx.shape[0]):
        for j in range(volumes.shape[0]):
            final_volume = posx2line(posx[i],volumes[j])
            for k in range(final_volume.shape[0]):
                if(i == 0 and j == 0 and k == 0):
                    plot_line(ax, final_volume[k][0], final_volume[k][1], label="pick pose")
                else :
                    plot_line(ax, final_volume[k][0], final_volume[k][1], label=None)
    for i in range(robot.shape[0]):
        plot_line(ax, robot[i][0], robot[i][1], label=None, color = 'red')

def pick_plotCallback(data):

    global plot_, obstacleG, verticesG, volumesG, posxG, servoing_pointG, robotG

    obstacleG = np.array(data.obstacle)
    obstacleG = obstacleG.reshape(-1,4,3)

    verticesG = np.array(data.vertices)
    verticesG = verticesG.reshape(-1,8,3)

    volumesG = np.array(data.volumes)
    volumesG = volumesG.reshape(-1,2,3)

    posxG = np.array(data.posx)
    posxG = posxG.reshape(-1,6)

    robotG = np.array(data.robot)
    robotG = robotG.reshape(4,2,3)

    plot_ = 'pick'

# 메인 코드
if __name__=="__main__":
    rospy.init_node('collision_detect_plot', anonymous=True)
    rospy.Subscriber('/collision_detect/servoing_plot', servoing_plot, servoing_plotCallback)
    rospy.Subscriber('/collision_detect/pick_plot', pick_plot, pick_plotCallback)
    print("waiting msg...")

    global plot_, obstacleG, verticesG, volumesG, posxG, servoing_pointG


    rate = rospy.Rate(10)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')


    plot_ = 'waiting'
    output_dir = '/home/lee/catkin_ws/src/plantfarm_ui/rc/'

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

        if(plot_== 'servoing'):

            plot_servoing(ax, obstacleG, verticesG, volumesG, posxG, servoing_pointG, robotG)

            ax.set_xlim(0, 500)
            ax.set_ylim(0, 500)
            ax.set_zlim(0, 500)


            file_path = os.path.join(output_dir, "servoing_plot.png")

            # 플롯을 파일로 저장
            plt.savefig(file_path)
            plt.show()

            plt.clf()
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            # 화면을 갱신하고 이벤트를 처리

            plot_ = 'waiting'

        if(plot_=='pick'):

            plot_pick(ax, obstacleG, verticesG, volumesG, posxG, robotG)

            ax.set_xlim(0, 500)
            ax.set_ylim(0, 500)
            ax.set_zlim(0, 500)


            file_path = os.path.join(output_dir, "pick_plot.png")

            # 플롯을 파일로 저장
            plt.savefig(file_path)
            plt.show()

            plt.clf()
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            # 화면을 갱신하고 이벤트를 처리

            plot_ = 'waiting'
        rate.sleep()
