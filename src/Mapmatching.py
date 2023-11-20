#!/usr/bin/python
# -*- coding: UTF-8 -*-

from CarRecord import *
from Arc import *
from math import *
import queue
import matplotlib.pyplot as plt
import sys
import time
import os
from datetime import datetime
import argparse


def DistanceBetween(geo1, geo2):
    return pow(pow(float(geo1[0]) - float(geo2[0]), 2) + pow(float(geo1[1]) - float(geo2[1]), 2), 0.5)

# Knowing the coordinates ABC of the three vertices, see if the angle ACB is an acute triangle
def CheckOxygon(A, C, B):
    a = DistanceBetween(C, B)
    b = DistanceBetween(A, C)
    c = DistanceBetween(A, B)
    if a**2 + b**2 < c**2:
        return False
    else:
        return True

def CalDirection(geo_list):
    if geo_list[-1][0] == geo_list[0][0]:
        if geo_list[-1][1] > geo_list[0][1]:
            return float(0)
        else:
            return float(180)
    else:
        slope = (geo_list[-1][1] - geo_list[0][1]) / (geo_list[-1][0] - geo_list[0][0])
        if geo_list[-1][0] > geo_list[0][0]: # If the ray is in the first or second quadrant
            return 90 - (atan(slope) / pi * 180) # Because the angle is the angle with the y-axis, it needs 90
        else: # If the ray is in the third or fourth quadran
            return 90 - (atan(slope) / pi * 180) + 180

# Input geo1 and geo2 as two points on the straight line y = kx + d, and find the projection of geo3 on the straight line (that is, the intersection point with the straight line after being perpendicular)
# If an obtuse triangle is formed, the vertical point is the one between geo1 and geo2 that is close to geo3
def CalProjection(geo1, geo2, geo3):
    geo1 = [float(ele) for ele in geo1]
    geo2 = [float(ele) for ele in geo2]
    geo3 = [float(ele) for ele in geo3]
    a = DistanceBetween(geo2, geo3)
    b = DistanceBetween(geo1, geo3)
    c = DistanceBetween(geo1, geo2)
    #Ottagon triangle, and geo3 is close to geo2, including the case where the point is on the extension line of the line segment
    if (a**2 + c**2) <= b**2: # Obtuse-angled triangle, with geo3 close to geo2, including the case where the point is on the extension of the line segment.
        return geo2
    elif b**2 + c**2 <= a**2: #Obtuse-angled triangle, with geo3 close to geo1, including the case where the point is on the extension of the line segment.
        return geo1
    elif a + b == c: # Indicate that the point is on the line segment.
        return geo3
    else:
        if geo1[0] == geo2[0]: # If the line is vertical
            return [geo1[0], geo3[1]]
        elif geo1[1] == geo2[1]: # If the line is horizontal
            return [geo3[0], geo1[1]]
        else:
            #y=kx+d, 
            #k represents the slope of the line. 
            # The slope (k) indicates the steepness of the line, 
            # #representing the rate at which 
            #y changes with respect to x. 
            # If the slope is positive, the line is ascending; 
            # if negative, it's descending; and if zero, 
            # the line is horizontal.
            k = (geo1[1] - geo2[1]) / (geo1[0] - geo2[0]) 
            d = geo1[1] - k * geo1[0] 
            x4 = (k * geo3[1] - k * d + geo3[0]) / (1 + k**2)
            y4 = k * x4 + d
            return [x4, y4]

def CalProjectionOfArc(arc, geo):
    length = len(arc.geometry_list)
    # First, find a virtual point. This point is located at an equal distance along the extension of the line connecting arc.geometry_list[-2] to arc.geometry_list[-1].
    virtual_p = [(2 * arc.geometry_list[-1][i] - arc.geometry_list[-2][i]) for i in range(2)]
    #Find the value of  i at which the triangle ACB becomes obtuse, where point A is car_record.geo, point C is arc.geometry_list[i], and point B is virtual_p.
    i = 0
    while i < length - 1 and CheckOxygon(geo, arc.geometry_list[i], virtual_p):
        i += 1
    # If the minimum value of i is 0 or len()−1, then calculate the perpendicular from the target point using the two points closest to it
    # Even if an obtuse-angled triangle scenario occurs, it will be handled in the calProjection function
    if i == 0:
        projection = CalProjection(arc.geometry_list[i], arc.geometry_list[i + 1], geo)
    else:  # If the perpendicular projection point is on the line segment" is the English translation for
        projection = CalProjection(arc.geometry_list[i - 1], arc.geometry_list[i], geo)
    return projection

# If geo2 represents two points on the line 
# y=kx+d, the distance from geo3 to the line is calculated. 
# If it forms an obtuse-angled triangle, 
# then the projection point is taken as the point closer 
# to geo3 among geo1 and the point in geo2, and the distance 
# is calculated from geo3 to that point.
def CalDistance(geo1, geo2, geo3):
    geo1 = [float(ele) for ele in geo1]
    geo2 = [float(ele) for ele in geo2]
    geo3 = [float(ele) for ele in geo3]
    a = DistanceBetween(geo2, geo3)
    b = DistanceBetween(geo1, geo3)
    c = DistanceBetween(geo1, geo2)
    if (a ** 2 + c ** 2) <= b ** 2:  # Obtuse-angled triangle, and geo3 is close to geo2, including the case where the point is on the extension of the line segment.
        return DistanceBetween(geo2, geo3) * 5 # 
    elif b ** 2 + c ** 2 <= a ** 2:  # A penalty of *5 is applied when the geo point moves outside.
        return DistanceBetween(geo1, geo3) * 5
    elif a + b == c:  # Indicate that the point is on the line segment
        return  0
    else:
        if geo1[0] == geo2[0]:  # If the line is vertical
            return abs(geo3[0] - geo1[0])
        elif geo1[1] == geo2[1]:  # If the line is horizontal
            return abs(geo3[1] - geo1[1])
        else:
            k = (geo1[1] - geo2[1]) / (geo1[0] - geo2[0])  
            d = geo1[1] - k * geo1[0]
            # Calculate distancd 
            dist = abs(k * geo3[0] + d - geo3[1]) / sqrt(1 + k**2) # 点到直线距离公式
            return dist

# f the vehicle's positioning data is far from the road or has an incorrect direction, there is no need to consider that particular arc.
def CheckNecessary(arc, car_record):
    # If the direction of the edge (edgh) is different from the direction of the car, then it is considered that the car is not on that road.
    if car_record.speed > 3 * 3.6:
        direction_road = CalDirection(arc.geometry_list)
        diff_angle = abs(car_record.direction - direction_road)
        if diff_angle > 180:
            diff_angle = 360 - diff_angle
        if diff_angle > 90:  # If the road direction is different from the car direction
            return False

    #  have calculated that the longest road in this map is 3494m. I am using a larger value, 10000, to prevent cases of oversight because the following 'if' statement only uses the coordinates of the first point on the road to calculate the distance from the target point.
    if DistanceActual(arc.geometry_list[0], car_record.geo) > 10000:
        return False

    # The subsequent 'if' statement calculates the shortest distance from all GPS points on this road to the target point. It returns True only if the minimum distance is less than 100m. We consider the car GPS accuracy to be 60m, and we are allowing a relaxation to 100m
    min_dist = float("inf")
    for geo1 in arc.geometry_list:
        tmp = DistanceActual(geo1, car_record.geo)
        if tmp < min_dist:
            min_dist = tmp
    if min_dist < 300:
        return True
    else:
        return False

def CalCost(arc, car_record):
    cost = float("inf")
    if CheckNecessary(arc, car_record):
        length = len(arc.geometry_list)
        # First, find a virtual point. This point is located at an equal distance along the extension of the line connecting arc.geometry_list[-2] to arc.geometry_list[-1].
        virtual_p = [(2 * arc.geometry_list[-1][i] - arc.geometry_list[-2][i]) for i in range(2)]
        # Find the value of i at which the triangle ACB becomes obtuse, where point A is car_record.geo, point C is arc.geometry_list[i], and point B is virtual_p.
        i = 0
        while i < length - 1 and CheckOxygon(car_record.geo, arc.geometry_list[i], virtual_p):
            i += 1
        # If the minimum value of  i is 0  len()−1, then calculate the perpendicular from the target point using the two points closest to it.
        # Even if an obtuse-angled triangle scenario occurs, it will be handled in the calProjection function.
        if i == 0:
            dist = CalDistance(arc.geometry_list[i], arc.geometry_list[i + 1], car_record.geo)
            direction_road = CalDirection(arc.geometry_list[0:2])
        else: # If the perpendicular projection point is on the line segment" is the English translation for
            dist = CalDistance(arc.geometry_list[i - 1], arc.geometry_list[i], car_record.geo)
            direction_road = CalDirection(arc.geometry_list[i - 1: i + 1])
        diff_angle = abs(car_record.direction - direction_road)
        if diff_angle > 180:
            diff_angle = 360 - diff_angle
        if car_record.speed < 1 * 3.6: #If the vehicle speed is less than 1, the weight of the direction is set to 0.
            angle_ratio = 0
        elif car_record.speed < 10 * 3.6: #If the vehicle speed is between 1 m/s and 10 m/s, as the speed increases, the weight of the direction gradually increases to 0.54
            angle_ratio = (car_record.speed / 3.6 - 1) * 0.06
        else: #If the speed is greater than 10 m/s, the weight of the direction remains unchanged at 0.54.
            angle_ratio = 0.54
        cost = (dist / 0.00046 * 39.3) * (1 - angle_ratio) + (1.5 * diff_angle) * angle_ratio
    return cost

def CalMinCostArcID(car_record):
    min_cost = float("inf")
    id = 0
    for arc in arc_objects:
        cost = CalCost(arc, car_record)
        if cost < min_cost:
            min_cost = cost
            id = arc.id
    return id

#t1 - t2 yyyyMMddHHmmss
def SecondsBetween(t1, t2):
    if int(t1) > int(t2):
        big = t1
        small = t2
    else:
        big = t2
        small = t1
    big = time.strptime(big, "%Y%m%d%H%M%S")
    small = time.strptime(small, "%Y%m%d%H%M%S")
    big = datetime(big[0], big[1], big[2], big[3], big[4], big[5])
    small = datetime(small[0], small[1], small[2], small[3], small[4], small[5])
    return (big - small).seconds

#From the first item of the road data provided, 
# we can know that the latitude and longitude of 0.00046 
# corresponds to the actual distance of 39.3m.

def DistanceActual(geo1, geo2):
    dist = DistanceBetween(geo1, geo2)
    return dist * 9.745339101028061e4

# Draw the single line in red and blue
def PlotMapRoad(arc_objects):
    record = {}
    for arc in arc_objects:
        ploted = False
        if record.has_key(arc.to_node):
            for ele in record[arc.to_node]:
                if ele == arc.from_node:
                    ploted = True
        nodeX = [geo[0] for geo in arc.geometry_list]
        nodeY = [geo[1] for geo in arc.geometry_list]
        if ploted == True:
            plt.plot(nodeX, nodeY, color='r', marker='.', alpha=0.3)
            plt.text(nodeX[0], nodeY[0], arc.id, alpha=0.3)
        else:
            plt.plot(nodeX, nodeY, color='#7f7f7f', marker='.', alpha=0.3)
            plt.text(nodeX[0], nodeY[0], arc.id, alpha=0.3)

        if record.has_key(arc.from_node):
            record[arc.from_node].append(arc.to_node)
        else:
            record[arc.from_node] = [arc.to_node]

#The input arc_cover is a list of arc_id and cover, and the 
# function outputs the total length of roads contained in this arc_cover
def CalLenCover(arc_cover):
    travel_distance = 0
    for k in range(len(arc_cover)):
        travel_distance += arc_objects[arc_cover[k][0] - 1].len * arc_cover[k][1]
    return round(travel_distance,1)

# 输入arc_list是arc_id的列表，该函数输出这个arc_list包含的道路总长
def CalLen(arc_list):
    travel_distance = 0
    for k in range(len(arc_list)):
        travel_distance += arc_objects[arc_list[k] - 1].len
    return round(travel_distance,1)

def findMinDist(dist, collected):
    min_dist = float("inf")
    min_key = -1
    for key in dist:
        if dist[key] < min_dist and (not collected.__contains__(key)):
            min_dist = dist[key]
            min_key = key
    return min_key

# To find the shortest distance between two vertices using a single-source shortest path algorithm, you can use Dijkstra's algorithm.
# archlist
def dijkstra(graph, start_arc_id, end_arc_id):
    if start_arc_id == end_arc_id:
        return [start_arc_id]
    dist = {}
    path = {}
    collected = {}
    for key in graph:
        dist[key] = float("inf")
        for arc in graph[key]:
            if not dist.__contains__(arc.to_node):
                dist[arc.to_node] = float("inf")
    from_node = arc_objects[start_arc_id - 1].to_node
    to_node = arc_objects[end_arc_id - 1].from_node
    if graph.__contains__(from_node):
        for arc in graph[from_node]:
            dist[arc.to_node] = arc.len
            path[arc.to_node] = from_node
    dist[from_node] = 0
    collected[from_node] = True
    while True:
        node = findMinDist(dist, collected)
        if node == -1:
            break
        collected[node] = True
        if graph.__contains__(node):
            for arc in graph[node]:
                if not collected.__contains__(arc.to_node):
                    if dist[arc.to_node] > dist[node] + arc.len:
                        dist[arc.to_node] = dist[node] + arc.len
                        path[arc.to_node] = node
    arc_list = [end_arc_id]
    while path.__contains__(to_node):
        for arc in graph[path[to_node]]:
            if arc.to_node == to_node:
                arc_list.append(arc.id)
        to_node = path[to_node]
    arc_list.append(start_arc_id)
    arc_list = arc_list[::-1]
    return arc_list

def BFSFindPathArc(arc, car_record, dist):
    min_cost = 60
    arc_list = []
    min_arc_id = 0
    visited = {}
    graph_around = {}
    q = queue.Queue()
    q.put(arc)
    visited[arc.id] = True
    while not q.empty():
        arc1 = q.get()
        # 生成arc附近连通的道路的图，后面用来找最短路径
        if graph_around.__contains__(arc1.from_node):
            graph_around[arc1.from_node].append(arc1)
        else:
            graph_around[arc1.from_node] = [arc1]
        cost = CalCost(arc1, car_record)
        if cost < min_cost:
            min_cost = cost
            min_arc_id = arc1.id
        if map_graph.__contains__(arc1.to_node):
            for arc2 in map_graph[arc1.to_node]:
                if not visited.__contains__(arc2.id):
                    if DistanceBetween(arc.geometry_list[-1], arc2.geometry_list[0]) < dist:
                        q.put(arc2)
                        visited[arc2.id] = True
    if min_arc_id != 0:
        arc_list = dijkstra(graph_around, arc.id, min_arc_id)
    return arc_list

# 计算点geo在arc上面的cover, flag="forward"时计算geo点垂点向前占arc的比例，flag="backward"时相反
def CalCover(arc, geo, flag):
    length = len(arc.geometry_list)
    virtual_p = [(2 * arc.geometry_list[-1][i] - arc.geometry_list[-2][i]) for i in range(2)]
    # 找从哪个i开始，三角形ACB变成钝角，其中点A为car_record.geo,点C为arc.geometry_list[i]，点B为virtual_p
    i = 0
    while i < length - 1 and CheckOxygon(geo, arc.geometry_list[i], virtual_p):
        i += 1
    # If the minimum value of i is 0 or len() - 1, then take the two points closest to the target point to calculate the perpendicular point
    # Even if obtuse-angled triangle situations occur, they will be handled in the calProjection function.
    if i == 0:
        projection = CalProjection(arc.geometry_list[i], arc.geometry_list[i + 1], geo)
    else:  # If the perpendicular point lies on the line segment.
        projection = CalProjection(arc.geometry_list[i - 1], arc.geometry_list[i], geo)
    cover = 0
    if flag == "forward":
        cover = round(DistanceBetween(projection, arc.geometry_list[-1]) \
                 / DistanceBetween(arc.geometry_list[0], arc.geometry_list[-1]), 2)
    elif flag == "backward":
        cover = round(DistanceBetween(projection, arc.geometry_list[0]) \
                      / DistanceBetween(arc.geometry_list[0], arc.geometry_list[-1]), 2)
    return cover

def CarRecordsDivide(car_i):
    index_start = sum(car_id_count[:car_i])
    index_end = sum(car_id_count[:car_i+1])
    if index_end - index_start == 1:
        car_i_start_end_index_list = [[index_start, index_end]]
    car_i_start_end_index_list = []
    index_j_start = index_start
    for i in range(index_start + 1, index_end):
        if SecondsBetween(car_record_objects[i - 1].time, car_record_objects[i].time) > 15*60:
            car_i_start_end_index_list.append([index_j_start, i])
            index_j_start = i
    car_i_start_end_index_list.append([index_j_start, index_end])
    return  car_i_start_end_index_list

def main():
    for car_i in range(0, car_num):
        print("Calculating car track of " + str(car_i + 1) + ", total " + str(car_num))
        output_file = open(result_file_name, 'a')
        # debug 可视化
        if PLOT == True:
            fig = plt.figure(car_i + 1)
        # If the time between two consecutive location points exceeds 15 minutes, consider splitting the trajectory into two segments. In other words, do not calculate the roads passed between these two location points
        car_i_start_end_index_list = CarRecordsDivide(car_i)
        for car_j in range(len(car_i_start_end_index_list)):
            car_ij_start_index = car_i_start_end_index_list[car_j][0] # The first location data of the j-th segment of the i-th vehicle's data.
            car_ij_end_index = car_i_start_end_index_list[car_j][1] # The last location data of the j-th segment of the i-th vehicle

            # arc_path存储着car_ij_start_index到car_ij_end_index之间经过的道路列表
            arc_path = []
            # Because the direction of the car is not reliable when just starting, the average direction and coordinates of several points within 10 meters are taken
            j = car_ij_start_index
            while j < car_ij_end_index:
                if DistanceActual(car_record_objects[j].geo, car_record_objects[car_ij_start_index].geo) > 10:
                    break
                j += 1
            car_record_list = car_record_objects[car_ij_start_index : j]
            car_geo_list = [car_record.geo for car_record in car_record_list]
            car_x = sum([geo[0] for geo in car_geo_list]) / len(car_geo_list)
            car_y = sum([geo[1] for geo in car_geo_list]) / len(car_geo_list)
            if len(car_geo_list) == 1:
                car_direction = car_record_list[0].direction
            else:
                car_direction = CalDirection(car_geo_list)
            car_record = CarRecord([0,0, car_x, car_y, 3*3.6+1, car_direction])
            arc_path.append(CalMinCostArcID(car_record))
            # Calculate the remaining, based on the previously calculated results, to find which road a road belongs to from the roads connected to it
            while j < car_ij_end_index:
                car_record = car_record_objects[j]
                arc_list = BFSFindPathArc(arc_objects[arc_path[-1] - 1], car_record, DistanceBetween(car_record.geo, arc_objects[arc_path[-1] - 1].geometry_list[-1]))
                if len(arc_list) != 0:
                    arc_list.pop(0)
                    if len(arc_list) != 0:
                        arc_path = arc_path + arc_list
                else:
                    if len(arc_path) > 1:
                        arc_list = BFSFindPathArc(arc_objects[arc_path[-2] - 1], car_record, DistanceBetween(car_record.geo, arc_objects[arc_path[-2] - 1].geometry_list[-1]))
                        if len(arc_list) != 0:
                            arc_list.pop(0)
                            if len(arc_list) != 0:
                                arc_path.pop(-1)
                                arc_path = arc_path + arc_list
                        else:
                            arc_id = CalMinCostArcID(car_record)
                            if arc_id != 0 and arc_id != arc_path[-1]:
                                arc_path.append(arc_id)
                    else:
                        arc_id = CalMinCostArcID(car_record)
                        if arc_id != 0 and arc_id != arc_path[-1]:
                            arc_path.append(arc_id)
                j += 1

            # debug 可视化
            if PLOT == True:
                for arc_id in arc_path: # draw the roads that the car has passed through.
                    nodex = [ele[0] for ele in arc_objects[arc_id - 1].geometry_list]
                    nodey = [ele[1] for ele in arc_objects[arc_id - 1].geometry_list]
                    plt.plot(nodex, nodey, color='#7f7f7f', marker='.', alpha=0.5)
                    plt.text((nodex[0] + nodex[1])/2, (nodey[0] + nodey[1])/2, arc_id, color='r', alpha=0.3)
                for j in range(car_ij_start_index, car_ij_end_index): # draw the car's GPS coordinate points
                    plt.scatter(car_record_objects[j].geo[0], car_record_objects[j].geo[1], marker='.', color='b', s=40, alpha=0.3)

            ## The following code calculates which arcs the segments between car_ij_start_index and car_ij_end_index pass through in the arc_path.
            if len(arc_path) == 1:
                projection_before = CalProjectionOfArc(arc_objects[arc_path[0] - 1], car_record_objects[car_ij_start_index].geo)
                for j in range(car_ij_start_index+1, car_ij_end_index):
                    # 去除重复GPS点
                    if car_record_objects[j].time == car_record_objects[j - 1].time:
                        continue
                    projection_now = CalProjectionOfArc(arc_objects[arc_path[0] - 1], car_record_objects[j].geo)
                    arc_cover = [[arc_path[0], round(DistanceActual(projection_before, projection_now) / arc_objects[arc_path[0] - 1].len, 4)]]
                    projection_before = projection_now # 更新projection_before的值
                    # 把arc_cover转换成要求格式，以便于输出
                    geo_output = []
                    for ele in arc_cover:
                        geo_output.append(':'.join([str(e) for e in ele]))
                    geo_output = '|'.join(geo_output)
                    # output
                    output_line = [str(car_i + 1), car_record_objects[j - 1].time, car_record_objects[j].time, str(CalLenCover(arc_cover)), \
                                   str(SecondsBetween(car_record_objects[j - 1].time,car_record_objects[j].time)), geo_output]
                    output_file.write(",".join(output_line) + "\n")
            else:
                # arc_path_i_before代表前一个arc在arc_path中的索引
                arc_path_i_before = 0
                for j in range(car_ij_start_index+1, car_ij_end_index):
                    # 去除重复GPS点
                    if car_record_objects[j].time == car_record_objects[j - 1].time:
                        continue
                    # dist 下一个定位点与上一个定位点之间的距离
                    dist = DistanceBetween(car_record_objects[j].geo, arc_objects[arc_path[arc_path_i_before] - 1].geometry_list[-1])
                    # arc_path_i_reachable表示与arc_path_i_before的距离小于dist的arc_path的索引
                    arc_path_i_reachable = arc_path_i_before + 1
                    while arc_path_i_reachable < len(arc_path):
                        if DistanceBetween(arc_objects[arc_path[arc_path_i_reachable] - 1].geometry_list[0], arc_objects[arc_path[arc_path_i_before] - 1].geometry_list[-1]) < dist:
                            arc_path_i_reachable += 1
                        else:
                            break
                    # 这样arc_path中arc_path_i_before到arc_path_i_reachable之间的arc就是j可能属于的arc
                    # 下面这段代码找arc_path_i_before到arc_path_i_reachable之间，j到底属于哪一条arc
                    min_cost = float('inf')
                    min_arc_path_i = arc_path_i_before
                    for arc_path_i in range(arc_path_i_before, arc_path_i_reachable):
                        cost = CalCost(arc_objects[arc_path[arc_path_i] - 1], car_record_objects[j])
                        if cost < min_cost:
                            min_cost = cost
                            min_arc_path_i = arc_path_i
                    # 找到min_arc_path_i是j属于的arc_path中的arc的索引
                    arc_path_output = arc_path[arc_path_i_before:min_arc_path_i+1] # 从j-1到j经过的arc列表
                    arc_path_output_cover = [[arc_id, 1.0] for arc_id in arc_path_output] # 从j-1到j经过的arc列表，带cover
                    # 计算首尾的cover
                    if len(arc_path_output_cover) == 1:
                        projection1 = CalProjectionOfArc(arc_objects[arc_path_output_cover[0][0] - 1], car_record_objects[j].geo)
                        projection2 = CalProjectionOfArc(arc_objects[arc_path_output_cover[0][0] - 1], car_record_objects[j - 1].geo)
                        arc_path_output_cover[0][1] = round(DistanceActual(projection1, projection2) / arc_objects[arc_path_output_cover[0][0] - 1].len, 4)
                    else:
                        projection1 = CalProjectionOfArc(arc_objects[arc_path_output_cover[0][0] - 1], car_record_objects[j - 1].geo)
                        projection2 = CalProjectionOfArc(arc_objects[arc_path_output_cover[-1][0] - 1], car_record_objects[j].geo)
                        arc_path_output_cover[0][1] = round(DistanceActual(projection1, arc_objects[arc_path_output_cover[0][0] - 1].geometry_list[-1]) \
                                                            / arc_objects[arc_path_output_cover[0][0] - 1].len, 4)
                        arc_path_output_cover[-1][1] = round(DistanceActual(projection2, arc_objects[arc_path_output_cover[-1][0] - 1].geometry_list[0]) \
                                                            / arc_objects[arc_path_output_cover[-1][0] - 1].len, 4)
                    # geo_output为处理arc_path_output_cover的数据，以便于输出
                    geo_output = []
                    for ele in arc_path_output_cover:
                        geo_output.append(':'.join([str(e) for e in ele]))
                    geo_output = ';'.join(geo_output)
                    # 输出
                    output_line = [str(car_i + 1), car_record_objects[j - 1].time, car_record_objects[j].time,str(CalLenCover(arc_path_output_cover)), \
                                   str(SecondsBetween(car_record_objects[j - 1].time, car_record_objects[j].time)), geo_output]
                    output_file.write(",".join(output_line) + "\n")
                    # 更新arc_path_i_before的值
                    arc_path_i_before = min_arc_path_i
        output_file.close()
        # debug 可视化图保存
        if PLOT == True:
            slash_index = result_file_name.rfind('/')
            if slash_index != -1:
                pic_dir = result_file_name[0:slash_index] + '/pic/'
            else:
                pic_dir = 'pic/'
            if not os.path.exists(pic_dir):
                os.mkdir(pic_dir)
            ax = plt.gca()
            ax.set_aspect(1)
            fig.savefig(pic_dir + str(car_i + 1))
            fig.clf()
            plt.close()

    return



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Mapmatching need to have 3 parameters")
    parser.add_argument("--maps", required=True, help="maps that is used to represent map information")
    parser.add_argument("--gpsdata", required=True, help="Data represents a GPS coordinate and orientation")
    parser.add_argument("--output", required=True, help="Cvs output folder")
    parser.add_argument("--plot", action="store_true",required=False, help="Plot the output")
    args = parser.parse_args()
    map_file = open(args.maps)
    map_lines = map_file.readlines()
    map_lines.pop(0)  # Remove the first line, as it serves as a description and does not contain actual data
    # The arc_objects store instances of the Arc object for all edges. The choice of naming it 'Arc' instead of 'Edge' is because 'Arc' represents directed edges, while 'Edge' typically denotes undirected edges.
    # Note that the elements in the arc_objects list have Arc IDs greater than their corresponding indices by 1, as the Arc IDs start from 1, while the list indices start from 0
    arc_objects = [Arc(argv.strip().split(',')) for argv in map_lines]

    car_file = open(args.gpsdata)
    car_lines_str = car_file.readlines()
    # car_record_objects stores every GPS location data for the car
    car_record_objects = [CarRecord(line_str.strip().split(',')) for line_str in car_lines_str]
    # car_id_coun store gps locatiomn
    car_id = [car_record.car_id for car_record in car_record_objects]
    car_num = max(car_id)
    car_id_count = [car_id.count(i + 1) for i in range(car_num)]  # 100辆车

    # Generating the adjacency list for a graph is a common way to represent the connections between vertices
    map_graph = {}
    for arc in arc_objects:
        if map_graph.__contains__(arc.from_node):
            map_graph[arc_objects[arc.id - 1].from_node].append(arc)
        else:
            map_graph[arc_objects[arc.id - 1].from_node] = [arc]

    # output the file name
    result_file_name = args.output
    if os.path.exists(result_file_name):
        os.remove(result_file_name)

    # whether to draw a graph
    PLOT = args.plot
    start = time.time()
    main()
    end = time.time()
    print("Spent " + str(int(end - start)) + " seconds")