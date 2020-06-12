#!/usr/bin/env python

# from node import Node
import colorsys
import copy
import math
from typing import List, NoReturn, Tuple

from PIL import Image


class Node:

    def __init__(self, x, y, h=0.0, g=0.0, f=0.0, prev=None):
        self.x = x
        self.y = y

        self.prev = prev

        # f(n) = h(n) + g(n)
        self.f = f
        # h(v)= √ ( (v.x − goal.x) ** 2 + (v.y − goal.y) ** 2 ).
        self.h = h
        self.g = g

    def euclidean_distance(self, goal_node):
        """
        Method to compute distance from current position to the goal
        @arg	goal 	Node object with x, y, theta
        @returns 	euclidean distance from current point to goal
        """
        return math.sqrt(((goal_node.x - self.x) ** 2) + ((goal_node.y - self.y) ** 2))


class VirtualPath:

    def __init__(self, grid_map: List[List[int]] = []):
        self.grid_map = grid_map

        print("Map: %s" % grid_map)

        self.start = ()
        self.end = ()

        self.reachable = {}
        self.explored = {}

    def set_map(self, grid_map: List[List[int]]) -> NoReturn:
        self.grid_map = grid_map

    def search(self, start_point: Tuple[int, int], end_point: Tuple[int, int]):

        print("Start search")

        if start_point == end_point:
            return []
        if not self.grid_map:
            return []

        x, y = start_point
        x_end, y_end = end_point
        next_node = Node(x, y)
        goal_node = Node(x_end, y_end)

        self.reachable[start_point] = next_node

        while self.reachable:

            self.check_nbrs(next_node, goal_node)

            if not self.reachable:
                print("We cannot found path :(")
                return

            print("self.reachable: %s" % self.reachable.keys())

            next_node = self.get_node_min_cost()

            print(
                "Next Node %s:%s, f:%s, h:%s, g:%s" % (next_node.x, next_node.y, next_node.f, next_node.h, next_node.g))
            print("")

            # iterations = iterations + 1
            if (next_node.x, next_node.y) == (goal_node.x, goal_node.y):
                break

        print("Final node coord x:%s, y:%s, path coast:%s" % (next_node.x, next_node.y, next_node.g))

        final_path = self.draw_path(next_node)

        found_path_on_map = copy.deepcopy(self.grid_map)
        for node in final_path:
            found_path_on_map[node.y][node.x] = '*'

        print("Map:")
        for row in self.grid_map:
            for col in row:
                print(str(col), end=" | ")
            print("")

        print("Final path:")
        for row in found_path_on_map:
            for col in row:
                print(str(col), end=" | ")
            print("")

        # print("Final path: %s" % found_path_on_map)

    def check_nbrs(self, current_node: Node, goal_node: Node):
        x = current_node.x
        y = current_node.y

        del self.reachable[(x, y)]
        self.explored[(x, y)] = current_node

        nbrs = self.find_nbrs(current_node)

        for coord, rate in nbrs.items():

            if self.reachable.get(coord):
                nbr_node = self.reachable.get(coord)
            else:
                nbr_node = Node(*coord)
                self.reachable[coord] = nbr_node

            h = nbr_node.euclidean_distance(goal_node)
            cost = self.calculate_cost(x, nbr_node.x, y, nbr_node.y, rate)
            g = current_node.g + cost
            f = h + g
            if not nbr_node.f or f < nbr_node.f:  # if node is new OR new path is low than path of node
                print("nbr %s:%s, f:%s, g:%s, h:%s" % (nbr_node.x, nbr_node.y, f, g, h))
                nbr_node.h = h
                nbr_node.g = g
                nbr_node.f = f
                nbr_node.prev = current_node

    def find_nbrs(self, node):
        x = node.x
        y = node.y
        steps = (-1, 0, 1)
        nbrs = {}
        for i in steps:
            y1 = y + i
            for j in steps:
                x1 = x + j

                if self.explored.get((x1, y1)) is not None:
                    continue
                if (x1, y1) == (x, y):
                    continue
                if x1 < 0 or y1 < 0:
                    continue
                try:
                    rate = self.grid_map[y1][x1]  # rate of surface, will be dynamic
                    if not rate:  # if rate = 0
                        continue
                except IndexError:
                    continue
                # print("Coord: %s,%s" % (x1,y1))
                nbrs[(x1, y1)] = rate
        return nbrs

    @staticmethod
    def calculate_cost(x: int, x1: int, y: int, y1: int, rate=1) -> float:
        default_cost = 1
        if x != x1 and y != y1:
            cost = default_cost * rate * math.sqrt(2)  # a * √2
        else:
            cost = default_cost * rate
        return cost

    def get_node_min_cost(self) -> Node:
        key = min(self.reachable, key=lambda k: self.reachable[k].f)
        return self.reachable[key]

    @staticmethod
    def draw_path(end_node: Node) -> List[Node]:
        current = end_node
        path = []
        while current is not None:
            path.append(current)
            current = current.prev
        return path


class MapHandler:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.grid_map = []
        self.blue_point: Tuple[int, int] = ()
        self.green_point: Tuple[int, int] = ()
        self.yellow_point: Tuple[int, int] = ()
        self.red_point: Tuple[int, int] = ()

    def get_map_from_image(self):
        map_image = Image.open('./map.png')
        self.width, self.height = map_image.size
        pixels = map_image.load()
        self.grid_map = []
        for y in range(self.width):
            row = []
            for x in range(self.height):
                rgb = pixels[x, y][:3]

                self.set_points(x, y, rgb)

                if self.is_black(rgb):
                    row.append(0)
                elif self.is_white(rgb):
                    row.append(1)
                else:
                    row.append(self.gray_rate(rgb))
            self.grid_map.append(row)
        return self.grid_map

    def set_points(self, x, y, rgb_color=()):
        if self.is_green(rgb_color):
            self.green_point = (x, y)
        elif self.is_blue(rgb_color):
            self.blue_point = (x, y)
        elif self.is_yellow(rgb_color):
            self.yellow_point = (x, y)
        elif self.is_red(rgb_color):
            self.red_point = (x, y)

    @staticmethod
    def gray_rate(rgb):
        """
        Return rate of gray.
        When Lightness closer to white the Rate is lower
        Lightness = 80, Rate = 2
        Lightness = 20, Rate = 8
        Lightness < 10, Rate = 0 is black
        Lightness = 100, Rate = 1 is white
        """
        h, s, l = MapHandler.get_hsl(rgb)
        if s > 2:  # not a gray, is some color
            return 1
        if l < 10:  # is black
            return 0
        if l == 100:  # is white
            return 1
        return 10 - int(l // 10)  # convert 10 - 8 = 2

    @staticmethod
    def is_black(rgb):
        r, g, b = rgb[:3]
        return True if r < 20 and g < 20 and b < 20 else False

    @staticmethod
    def is_white(rgb):
        return True if rgb[:3] == (255, 255, 255) else False

    @staticmethod
    def is_green(rgb):
        h, s, l = MapHandler.get_hsl(rgb)
        if l == 100 or (l < 10 and s < 10):
            return False
        return True if 80 < h < 180 else False

    @staticmethod
    def is_blue(rgb):
        h, s, l = MapHandler.get_hsl(rgb)
        if l == 100 or (l < 10 and s < 10):
            return False
        return True if 180 < h < 270 else False

    @staticmethod
    def is_red(rgb):
        h, s, l = MapHandler.get_hsl(rgb)
        if l == 100 or (l < 10 and s < 10):
            return False
        return True if 350 < h < 360 or 0 < h < 10 else False

    @staticmethod
    def is_yellow(rgb):
        h, s, l = MapHandler.get_hsl(rgb)
        if l == 100 or (l < 10 and s < 10):
            return False
        return True if 50 < h < 80 else False

    @staticmethod
    def get_hsl(rgb) -> tuple:
        c = [x / 255.0 for x in rgb[:3]]
        h, l, s = colorsys.rgb_to_hls(*c)
        return h * 360, s * 100, l * 100


if __name__ == '__main__':

    # m = [
    #     [1, 1, 1, 0, 3, 1, 1],
    #     [1, 0, 0, 6, 2, 1, 1],
    #     [1, 1, 0, 0, 1, 0, 1],
    #     [1, 1, 0, 1, 1, 0, 1],
    #     [1, 1, 1, 1, 1, 1, 1],
    # ]

    # Result like this
    # 1 | * | * | 0 | 1 | 1 | 1 |
    # * | 0 | 0 | * | * | * | 1 |
    # 1 | 1 | 0 | 0 | 1 | 0 | * |
    # 1 | 1 | 0 | 1 | 1 | 0 | * |
    # 1 | 1 | 1 | 1 | 1 | 1 | 1 |

    mh = MapHandler()
    m = mh.get_map_from_image()

    print("Grid map: %s" % m)

    p = VirtualPath(grid_map=m)
    sp = mh.blue_point
    ep = mh.green_point

    print("start point: %s:%s" % sp)
    print("end point: %s:%s" % ep)

    if not sp or not ep:
        print("Please set the start and end points")
        exit(1)

    p.search(sp, ep)

    # p.search((2, 0), (6, 2))
