#!/usr/bin/python3
class Vec3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.eps = 0.000001
    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __mul__(self, other):
        return Vec3(self.x * other.x, self.y * other.y, self.z * other.z)
    def __eq__(self, other):
        return abs(self.x - other.x) < self.eps and abs(self.y - other.y) < self.eps and abs(self.z - other.z) < self.eps
    def mul(self, other):
        return Vec3(self.x * other, self.y * other, self.z * other)
    def norm(self):
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** 0.5
    def normalize(self):
        norm = self.norm()
        if norm == 0:
            return Vec3(0, 0, 0)
        return Vec3(self.x / norm, self.y / norm, self.z / norm)
    def __str__(self):
        return (str(self.x)+' '+str(self.y)+' '+str(self.z))
    def __repr__(self):
        return (str(self.x)+' '+str(self.y)+' '+str(self.z))

def intersect_line_segment_aabbox(line, bbox, debug=False):
    """
    intersection between line segment and 3d bounding box
    """
    eps = Vec3(0.0000001, 0.0000001, 0.0000001)
    if debug:
        print(line)
    line_start = line[0]
    line_end = line[1]
    
    bbox_min = bbox[0]
    bbox_max = bbox[1]
    line_dir = (line_end - line_start) + eps
    line_length = line_dir.norm()
    line_dir = line_dir.normalize()

    line_invdir = Vec3(1.0 / line_dir.x, 1.0 / line_dir.y, 1.0 / line_dir.z)
    tmin = (bbox_min - line_start) * line_invdir
    tmax = (bbox_max - line_start) * line_invdir
    t0 = max(max(min(tmin.x, tmax.x), min(tmin.y, tmax.y)), min(tmin.z, tmax.z))
    t1 = min(min(max(tmin.x, tmax.x), max(tmin.y, tmax.y)), max(tmin.z, tmax.z))
    if (t1 < 0):
        if debug:
            print('behind')
        return None
    if (t0 > t1):
        if debug:
            print('no ray intersection')
        return None
    if debug:
        print('ray intersect')
    if line_start.x > bbox_min.x and line_start.x < bbox_max.x and line_start.y > bbox_min.y and line_start.y < bbox_max.y and line_start.z > bbox_min.z and line_start.z < bbox_max.z:
        t = t1
    else:
        t = t0
    if t <= line_length+0.00001:
        if debug:
            print('line intersect')
        return line_start + line_dir.mul(t)
    else:  
        if debug:
            print('no line intersection')
        return None
if __name__ == '__main__':
    bbox = (Vec3(-1, -1, -1), Vec3(1, 1, 1))
    print('---1')
    if (intersect_line_segment_aabbox((Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5)), bbox)):
        raise Exception('wrong')
    print('---2')
    if not(intersect_line_segment_aabbox((Vec3(0, 0, 0), Vec3(1, 0, 0)), bbox) == Vec3(1, 0, 0)):
        raise Exception('wrong')
    print('---3')
    if not(intersect_line_segment_aabbox((Vec3(0, 0, 0), Vec3(-1, 0, 0)), bbox) == Vec3(-1, 0, 0)):
        raise Exception('wrong')
    print('---4')
    if (intersect_line_segment_aabbox((Vec3(-2, -2, -2), Vec3(-2, -2, 0)), bbox)):
        raise Exception('wrong')
    print('---5')
    if (intersect_line_segment_aabbox((Vec3(-2, -2, -2), Vec3(-3, -3, -3)), bbox)):
        raise Exception('wrong')
    print('---6')
    if not(intersect_line_segment_aabbox((Vec3(0, 0, 0), Vec3(-2, -2, -2)), bbox) == Vec3(-1, -1, -1)):
        raise Exception('wrong')
    print('---7')
    if not(intersect_line_segment_aabbox((Vec3(-2, -2, -2), Vec3(0, 0, 0)), bbox) == Vec3(-1, -1, -1)):
        raise Exception('wrong')
    print('---8')
    if not(intersect_line_segment_aabbox((Vec3(-2, -2, -2), Vec3(2, 2, 2)), bbox) == Vec3(-1, -1, -1)):
        raise Exception('wrong')