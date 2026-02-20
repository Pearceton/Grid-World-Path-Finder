import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation
import math

from utils import *
from grid import *
from algorithms import *

def gen_polygons(worldfilepath):
    polygons = []
    with open(worldfilepath, "r") as f:
        lines = f.readlines()
        lines = [line[:-1] for line in lines]
        for line in lines:
            polygon = []
            pts = line.split(';')
            for pt in pts:
                xy = pt.split(',')
                polygon.append(Point(int(xy[0]), int(xy[1])))
            polygons.append(polygon)
    return polygons

def point_in_polygon(point, polygon):
    x, y = point.x, point.y
    inside = False

    n = len(polygon)
    p1x, p1y = polygon[0].x, polygon[0].y

    for i in range(n + 1):
        p2x, p2y = polygon[i % n].x, polygon[i % n].y

        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside

        p1x, p1y = p2x, p2y

    return inside

def point_on_segment(p, a, b):
    cross = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y)
    if cross != 0:
        return False

    dot = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)
    if dot < 0:
        return False

    sq_len = (b.x - a.x)**2 + (b.y - a.y)**2
    if dot > sq_len:
        return False

    return True

def point_on_polygon_edge(point, polygon):
    n = len(polygon)
    for i in range(n):
        a = polygon[i]
        b = polygon[(i+1) % n]
        if point_on_segment(point, a, b):
            return True
    return False


def sld(node, problem):
    dx = node.STATE.x - problem.GOAL.x
    dy = node.STATE.y - problem.GOAL.y
    return math.sqrt(dx*dx + dy*dy)


def reconstruct_path(node):
    path = []
    while node:
        path.append(node.STATE)
        node = node.PARENT
    path.reverse()
    return path

class GridProblem:

    def __init__(self, initial, goal, enclosures, turfs):
        self.INITIAL = initial
        self.GOAL = goal
        self.enclosures = enclosures
        self.turfs = turfs

    def GOAL_TEST(self, state):
        return state == self.GOAL

    def ACTIONS(self, state):
        x, y = state.x, state.y
        actions = []

        moves = [
            (0, 1),    # Up
            (1, 0),    # Right
            (0, -1),   # Down
            (-1, 0)    # Left
        ]

        for dx, dy in moves:
            nx, ny = x + dx, y + dy

            if 0 <= nx < 50 and 0 <= ny < 50:
                next_point = Point(nx, ny)

                if not self.in_enclosure(next_point):
                    actions.append((dx, dy))

        return actions

    def RESULT(self, state, action):
        dx, dy = action
        return Point(state.x + dx, state.y + dy)

    def ACTION_COST(self, state, action, next_state):
        if self.in_turf(next_state):
            return 1.5
        return 1

    def in_enclosure(self, point):
        for polygon in self.enclosures:
            if point_in_polygon(point, polygon) or \
            point_on_polygon_edge(point, polygon):
                return True
        return False


    def in_turf(self, point):
        for polygon in self.turfs:
            if point_in_polygon(point, polygon):
                return True
        return False



if __name__ == "__main__":
    epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
    tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

    source = Point(8,10)
    dest = Point(43,45)

    fig, ax = draw_board()
    draw_grids(ax)
    draw_source(ax, source.x, source.y)
    draw_dest(ax, dest.x, dest.y)

    # Draw enclosure polygons
    for polygon in epolygons:
        for p in polygon:
            draw_point(ax, p.x, p.y)
    for polygon in epolygons:
        for i in range(len(polygon)):
            draw_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])
    # Draw turf polygons
    for polygon in tpolygons:
        for p in polygon:
            draw_green_point(ax, p.x, p.y)
    for polygon in tpolygons:
        for i in range(len(polygon)):
            draw_green_line(ax, [polygon[i].x, polygon[(i+1)%len(polygon)].x], [polygon[i].y, polygon[(i+1)%len(polygon)].y])

    problem = GridProblem(source, dest, epolygons, tpolygons)
    run_all(problem, lambda n: sld(n, problem))

    print("Choose algorithm:")
    print("1. A* Search")
    print("2. Breadth-First Search")
    print("3. Depth-First Search")
    print("4. Greedy Best-First Search")

    choice = input("Enter your choice (1-4): ")

    if choice == "1":
        goal_node, _ = a_star_search(problem, lambda n: sld(n, problem))
    elif choice == "2":
        goal_node, _ = breadth_first_search(problem)
    elif choice == "3":
        goal_node, _ = depth_first_search(problem)
    elif choice == "4":
        goal_node, _ = greedy_best_first_search(problem, lambda n: sld(n, problem))
    else:
        print("Invalid choice. Defaulting to Depth-First Search.")
        goal_node, _ = depth_first_search(problem)

    if goal_node:
        res_path = reconstruct_path(goal_node)
    else:
        res_path = []

    for i in range(len(res_path)-1):
        draw_result_line(ax, [res_path[i].x, res_path[i+1].x], [res_path[i].y, res_path[i+1].y])
        plt.pause(0.1)
    plt.show()
