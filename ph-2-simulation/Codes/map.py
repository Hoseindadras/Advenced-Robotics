import xml.etree.ElementTree as ET
import math
import matplotlib.pyplot as plt
from shapely.geometry import LineString,Point, Polygon

def calculate_rectangle_points(pose, geometry):


    x, y, z, roll, pitch, yaw = map(float, pose)
    length, width, height = map(float, geometry)

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    p1 = [
        x + (length * cos_yaw / 2) + (width * sin_yaw / 2),
        y + (length * sin_yaw / 2) - (width * cos_yaw / 2)
    ]

    p2 = [
        x + (length * cos_yaw / 2) - (width * sin_yaw / 2),
        y + (length * sin_yaw / 2) + (width * cos_yaw / 2)
    ]

    p3 = [
        x - (length * cos_yaw / 2) + (width * sin_yaw / 2),
        y - (length * sin_yaw / 2) - (width * cos_yaw / 2)
    ]

    p4 = [
        x - (length * cos_yaw / 2) - (width * sin_yaw / 2),
        y - (length * sin_yaw / 2) + (width * cos_yaw / 2)
    ]

    return [p1, p2, p3, p4]

def init_map(address):
    tree = ET.parse(address)
    root = tree.getroot()

    rects = []
    centers = []
    global_map_pose = None

    for model in root[0].iter('model'):
        try:
            for element in model:
                if element.tag == 'pose':
                    _global_map_pose = element.text.split(' ')
                    if _global_map_pose[0] != '0':
                        global_map_pose = _global_map_pose
                        print(global_map_pose)

            for link in model.iter('link'):
                pose, geometry = None, None

                for _pose in link.iter('pose'):
                    pose = _pose.text.split(' ')
                    break

                for collision in link.iter('collision'):
                    geometry = collision.find('.//geometry/box/size').text.split(' ')

                if pose and geometry:
                    rect_points = calculate_rectangle_points(pose, geometry)
                    rects.append(rect_points)
                    centers.append([float(pose[0]), float(pose[1])])

        except Exception as e:
            pass

    return rects, global_map_pose, map_boundry(centers)


def find_intersection(p1,p2,p3,p4):

    line1 = LineString([tuple(p1), tuple(p2)])
    line2 = LineString([tuple(p3), tuple(p4)])

    int_pt = line1.intersection(line2)
    if int_pt:
        point_of_intersection = int_pt.x, int_pt.y
        return point_of_intersection
    else:
        return False


def convert_point_to_line(rects):
    lines = []
    for points in rects:
        lines.append([ points[0] , points[1]] )
        lines.append([ points[1] , points[3]] )
        lines.append([ points[3] , points[2]] )
        lines.append([ points[2] , points[0]] )
    return lines


def add_offset(rects,offset):
    new_rects = []
    for points in rects: 
        new_rects.append(
            [  
                [points[0][0] + offset[0] , points[0][1] + offset[1]  ] ,
                [points[1][0] + offset[0] , points[1][1] + offset[1]  ] ,
                [points[2][0] + offset[0] , points[2][1] + offset[1]  ] ,
                [points[3][0] + offset[0] , points[3][1] + offset[1]  ] 
            ]
        )
    return new_rects


def convert_to_poly(rects):
    polygons = []
    for points in rects:
        polygons.append(Polygon(
            [tuple(points[0]) ,
            tuple(points[1]),
            tuple(points[3]),
            tuple(points[2])
            ]))
    return polygons

def check_is_collision(point , rects):
    p = Point(tuple(point))
    for rect in rects:
        if rect.contains(p):
            return True
    return False


def map_boundry(centers):
    X = []
    Y = [] 

    for item in centers:
        X.append(float(item[0]))
        Y.append(float(item[1]))

    return min(X),max(X),min(Y),max(Y)
    



def out_of_range(particle,offset,map_boundry):
    if particle[0] - offset[0] > map_boundry[1] or particle[0] - offset[0] < map_boundry[0]:
        return True
    elif particle[1] - offset[1] > map_boundry[3] or particle[1] - offset[1] < map_boundry[2]:
        return True
    else:
        return False

def plot_map(rects):
    for rect in rects:
        rect = list(zip(*rect))
        plt.plot(rect[1], rect[0], c='black')







