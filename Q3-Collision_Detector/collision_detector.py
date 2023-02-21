from typing import List, Tuple
import bisect
import math

def distance(p):
    return math.sqrt(p.x**2 + p.y**2 + p.z**2)

class Point:
    def __init__(self, x: float, y: float, z: float, box=None):
        self.x = x
        self.y = y
        self.z = z
        self.box = box
    def __hash__(self) -> int:
        return hash((self.x, self.y, self.z))
    def __eq__(self, __o: object) -> bool:
        return self.x == __o.x and self.y == __o.y and self.z == __o.z
    

# def distance(p: Point):
#     return math.sqrt(p.x**2 + p.y**2 + p.z**2)
        
class Box:
    def __init__(self, point1: Point, point2: Point):
        self.min_point = Point(min(point1.x, point2.x), min(point1.y, point2.y), min(point1.z, point2.z))
        self.max_point = Point(max(point1.x, point2.x), max(point1.y, point2.y), max(point1.z, point2.z))
        self.all_points = []
        for x in [self.min_point.x, self.max_point.x]:
            for y in [self.min_point.y, self.max_point.y]:
                for z in [self.min_point.z, self.max_point.z]:
                    self.all_points.append(Point(x, y, z, self))
        # self.max_point = max_point
        
def point_in_box(box: Box, point: Point) -> bool:
    return (box.min_point.x < point.x < box.max_point.x and
            box.min_point.y < point.y < box.max_point.y and
            box.min_point.z < point.z < box.max_point.z)
            
def box_intersect(box1: Box, box2: Box) -> bool:
    for p in box1.all_points:
        if point_in_box(box2, p):
            return True
    for p in box2.all_points:
        if point_in_box(box1, p):
            return True
    return False

def point_on_box_face(box: Box, point: Point) -> bool:
    return (point.x in [box.min_point.x, box.max_point.x] and box.min_point.y <= point.y <= box.max_point.y and box.min_point.z <= point.z <= box.max_point.z) or \
           (point.y in [box.min_point.y, box.max_point.y] and box.min_point.x <= point.x <= box.max_point.x and box.min_point.z <= point.z <= box.max_point.z) or \
           (point.z in [box.min_point.z, box.max_point.z] and box.min_point.x <= point.x <= box.max_point.x and box.min_point.y <= point.y <= box.max_point.y)
           
def optimized_function(boxes: List[Box], points: List[Point]):
    intersecting_boxes = set()
    idx_for_boxes= {b: i for i, b in enumerate(boxes)}
    boxes_for_points = [[] for _ in range(len(points))]
    # boxes_for_points = {p:[] for p in points}
    idx_for_points = {p: i for i, p in enumerate(points)}
    
    all_points = []
    for box in boxes:
        all_points.extend(box.all_points)
    for p in points:
        all_points.append(p)
    
    all_points.sort(key=distance)
    for p in all_points:
        print(f'{p.x} {p.y} {p.z}')
    for box in boxes:
        idx_min_point = bisect.bisect_left(all_points, distance(box.min_point), key=distance)
        idx_max_point = bisect.bisect_left(all_points, distance(box.max_point), key=distance)
        for idx in range(idx_min_point + 1, idx_max_point):
            if all_points[idx].box != None and all_points[idx].box != box and point_in_box(box, all_points[idx]):
                intersecting_boxes.add(frozenset({idx_for_boxes[box], idx_for_boxes[all_points[idx].box]}))
            elif all_points[idx].box == None and point_in_box(box, all_points[idx]):
                boxes_for_points[idx_for_points[all_points[idx]]].append(idx_for_boxes[box])
    
    res_intersecting_boxes = []
    for pair in intersecting_boxes:
        res_intersecting_boxes.append(list(pair))
    
    return res_intersecting_boxes, boxes_for_points

if __name__ == '__main__':
    box1 = Box(Point(1,1,1), Point(5,5,5))
    box2 = Box(Point(4,4,4), Point(10,10,10))
    box3 = Box(Point(9,9,9), Point(20,20,20))
    boxes = [box1, box2, box3]

    point1 = Point(3,3,3)
    point2 = Point(6,6,6)
    point3 = Point(16,16,16)
    points = [point1, point2, point3]

    pairs, boxes_for_points = optimized_function(boxes, points)
    for pair in pairs:
        print(f'pair: {pair[0], pair[1]}')
    for i, p_boxes in enumerate(boxes_for_points):
        print(f'p:{i} | boxes: {p_boxes}')
