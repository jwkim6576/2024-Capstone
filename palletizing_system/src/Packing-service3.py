# General system modules
import rospy
from py3dbp import Packer, Bin, Item, Painter

# Matplotlib for plotting
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# ROS service imports for palletizing system
from palletizing_system.srv import obj_info, obj_infoResponse

msgs = []
level = 1

def put_item_specs(obj_name, colors, dimension, level):
    # 객체 정보를 기반으로 py3dbp 아이템 사양 생성
    partno = obj_name
    name = obj_name
    type = "cube"  # 고정된 유형
    dimensions = dimension
    weight = 10  # 고정된 무게
    level = level  # 고정된 레벨
    loadbear = 100  # 고정된 하중
    updown = True  # 고정된 위아래
    color = colors[0]
    msgs.append([partno, name, type, dimensions, int(weight), int(level), int(loadbear), bool(updown), color])

def get_item_specs(msg):
    partno = msg[0] #'test1'각도로 쿼터니언을 계산하는 함수
    name = msg[1] #'test'
    type = msg[2] #'cube'
    dimensions = msg[3] #input("Enter item dimensions (width height depth, separated by spaces): ")
    weight = msg[4]
    level = msg[5]  # 1
    loadbear = msg[6] #100
    updown = msg[7] #True
    color = msg[8] #'darkgreen'
    return partno, name, type, dimensions, weight, level, loadbear, updown, color

def makeDictItem(item):
    # 아이템 정보를 딕셔너리 형태로 변환
    r = {
        "name": item.name,
        "Center position": item.getCenterPos(),
        "WHD": item.getDimension(),
        # "Orientation": item.getRoundedOrientation(4),
        # "Rotation matrix": item.getOrientation()
    }
    return r

def visualize_packing(ax, b):
    ax.cla()
    painter = Painter(b)
    painter.plotBoxAndItems(
        ax=ax,
        title=b.partno,
        alpha=0.8,
        write_num=False,
        fontsize=10
    )
    plt.draw()

def get_object_info_server(req):
    global level
    try:
        obj_names = req.x[0::2]
        colors = req.x[1::2]
        dimensions = req.y
        put_item_specs(obj_names, colors, dimensions, level)
        level += 1

        packer = Packer()
        box = Bin('Packing-Service', (150, 100, 600), 99, 0, 1)
        packer.addBin(box)
        for msg in msgs :
            item_specs = get_item_specs(msg)
           # print(item_specs)
            packer.addItem(Item(*item_specs))
        print("-------------")

        packer.pack(
            bigger_first=True,
            distribute_items=100,
            fix_point=True,
            check_stable=True,
            support_surface_ratio=0.75,
            number_of_decimals=0
        )

        res = obj_infoResponse()
        b = packer.bins[0]

        for item in b.items:
            print(makeDictItem(item))
            item_info = makeDictItem(item)
            res.width = item_info['WHD'][0]
            res.height = item_info['WHD'][1]
            res.depth = item_info['WHD'][2]
            res.x = item_info['Center position'][0] - 10
            res.y = item_info['Center position'][1] - 10
            res.z = item_info['Center position'][2]
         #   print("::::::::::::::::::::::")
        #print(":::::::::::", b.string(), ":::::::::::")

        visualize_packing(ax, b)

        return res
    except Exception as e:
        rospy.logerr(f"Error in service callback: {str(e)}")
        return obj_infoResponse()

if __name__ == '__main__':
    rospy.init_node('object_info_server')
    s = rospy.Service('get_obj_info_service', obj_info, get_object_info_server)  # 서비스 서버 생성
    rospy.loginfo("Ready to provide Packing information.")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plt.show()
    rospy.spin()
