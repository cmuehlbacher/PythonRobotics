"""
A Global bug algorithm

author: Clemens Muehlbacher
"""

from __future__ import print_function
import matplotlib.pyplot as plt
import math

show_animation = True

verification_calls = 0


class Node:
    def __init__(self, x, y, cost, pind, first_node=False):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        self.first_node = first_node

    def __str__(self):
        return "N " + str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind) + ":" + str(self.first_node)


class FollowWallNode(Node):
    def __init__(self, x, y, cost, pind, motion_index, first_node=False):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind
        self.motion_index = motion_index
        self.first_node = first_node

    def __str__(self):
        return "FW " + str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind) + "~" + str(self.motion_index)


class GlobalGraphNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self._out_connection = set()
        self._in_connection = set()

    def addOutConnection(self, edge):
        self._out_connection.add(edge)

    def addInConnection(self, edge):
        self._in_connection.add(edge)

    def getOutEdges(self):
        return self._out_connection

    def removeOutConnection(self, edge):
        self._out_connection.remove(edge)

    def removeInConnection(self, edge):
        self._in_connection.remove(edge)

class GlobalGraphEdge:
    def __init__(self, start_node, goal_node, cost):
        self.start_node = start_node
        self.goal_node = goal_node
        self.cost = cost

        self.start_node.addOutConnection(self)
        self.goal_node.addInConnection(self)

        self.checked = False

    def wasChecked(self):
        self.checked = True


class GlobalGraphExpansionNode:
    def __init__(self, node, cost, used_edge, previous_node):
        self.node = node
        self.cost = cost
        self.used_edge = used_edge
        self.previous_node = previous_node


def calc_fianl_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def global_a_start_planning(start_node, goal_node, xw, minx, miny):
    openset, closedset = set(), set()

    start = GlobalGraphExpansionNode(start_node, 0, None, None)

    openset, closedset = dict(), dict()
    openset[calc_index(start.node, xw, minx, miny)] = start

    while len(openset) > 0:
        # get best ranked item
        c_id = min(openset, key=lambda o: openset[o].cost + calc_h(goal_node, openset[o].node.x, openset[o].node.y))
        current = openset[c_id]

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand with all possible moves
        for out_going_edge in current.node.getOutEdges():

            node = GlobalGraphExpansionNode(out_going_edge.goal_node, current.cost + out_going_edge.cost,
                                            out_going_edge, current)

            n_id = calc_index(node.node, xw, minx, miny)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].used_edge = out_going_edge
                    openset[n_id].previous_node = current
            else:
                openset[n_id] = node

    # return found path
    goal = closedset[calc_index(goal_node, xw, minx, miny)]
    current_path_node = goal
    path = []
    while current_path_node is not None:
        if current_path_node.used_edge is not None:
            path.append(current_path_node.used_edge)
            current_path_node = current_path_node.previous_node
        else:
            break

    return path


def global_bug_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """
    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1, True)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)

    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart
    straight_line_solution = True

    while 1:
        # if len(openset) > 0:
        #     print("openset: ", end="")
        #     for o in openset.values():
        #         print(o, end='; ')
        #     print("")

        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
        #  print("current", current)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Found goal")
            if straight_line_solution:
                print("and it was a straight line")
            print("It took {} verification calls".format(verification_calls))
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # check if a greedy expand is possible
        min_index = 0
        should_follow_the_wall = True
        if not isinstance(current, FollowWallNode):
            min_cost = float('inf')
            for i in range(len(motion)):
                goal_cost = calc_h(ngoal, current.x + motion[i][0], current.y + motion[i][1])
                if min_cost > goal_cost:
                    min_cost = goal_cost
                    min_index = i

            min_node = Node(current.x + motion[min_index][0], current.y + motion[min_index][1],
                            current.cost + motion[min_index][2], c_id)
            min_node_id = calc_index(min_node, xw, minx, miny)

            if verify_node(min_node, obmap, minx, miny, maxx, maxy) and min_node_id not in closedset:
                n_id = calc_index(min_node, xw, minx, miny)

                if n_id in openset:
                    if openset[n_id].cost > min_node.cost:
                        openset[n_id].cost = min_node.cost
                        openset[n_id].pind = c_id
                else:
                    openset[n_id] = min_node

                should_follow_the_wall = False

        if should_follow_the_wall:
            straight_line_solution = False
            # follow the wall
            for i in range(len(motion)):
                node = FollowWallNode(current.x + motion[i][0], current.y + motion[i][1],
                                      current.cost + motion[i][2], c_id, i)
                if not verify_node(node, obmap, minx, miny, maxx, maxy):
                    continue

                n_id = calc_index(node, xw, minx, miny)
                if n_id in closedset:
                    continue
                # check that nearby is wall element
                wall_close_by = False
                for j in range(len(motion)):
                    check_node = Node(node.x + motion[j][0], node.y + motion[j][1],
                                      node.cost + motion[j][2], c_id)

                    if not verify_node(check_node, obmap, minx, miny, maxx, maxy):
                        wall_close_by = True

                    if wall_close_by:
                        break

                if not wall_close_by:
                    continue

                current_goal_cost = calc_h(ngoal, current.x, current.y)
                new_goal_cost = calc_h(ngoal, node.x, node.y)
                # stop following the wall as we improve
                if new_goal_cost < current_goal_cost:
                    node = Node(current.x + motion[i][0], current.y + motion[i][1],
                                current.cost + motion[i][2], c_id, isinstance(current, FollowWallNode))

                # add to open list or update the open set
                if n_id in openset:
                    if openset[n_id].cost > node.cost:
                        openset[n_id].cost = node.cost
                        openset[n_id].pind = c_id
                else:
                    openset[n_id] = node
            # change type of current node as we follow the wall
            current = FollowWallNode(current.x, current.y, current.cost, current.pind, 0, current.first_node)

            closedset[c_id] = current

        # show graph
        if show_animation:
            if not isinstance(current, FollowWallNode):
                plt.plot(current.x * reso, current.y * reso, "xc")
            else:
                plt.plot(current.x * reso, current.y * reso, "xg")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

    global_nodes_to_add = set()

    # traverse path and search for nodes to add
    path_node = ngoal
    while path_node.pind != -1:
        if path_node.first_node:
            global_nodes_to_add.add(path_node)

        new_path_node = closedset[path_node.pind]

        if not isinstance(path_node, FollowWallNode) and isinstance(new_path_node, FollowWallNode):
            global_nodes_to_add.add(path_node)

        path_node = new_path_node

    return global_nodes_to_add, straight_line_solution


def calc_h(ngoal, x, y):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):
    global verification_calls
    verification_calls += 1

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[int(node.x)][int(node.y)]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = int(round(maxx - minx))
    ywidth = int(round(maxy - miny))
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(ywidth)] for i in range(xwidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 20.0  # [m]
    sy = 10.0  # [m]
    gx = 20.0  # [m]
    gy = 100.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox, oy = [], []

    for i in range(40):
        ox.append(i)
        oy.append(0.0)
    for i in range(110):
        ox.append(40.0)
        oy.append(i)
    for i in range(41):
        ox.append(i)
        oy.append(110.0)
    for i in range(111):
        ox.append(0.0)
        oy.append(i)
    for i in range(10, 90):
        ox.append(10.0)
        oy.append(i)
    for i in range(10, 90):
        ox.append(30.0)
        oy.append(i)
    for i in range(10, 30):
        ox.append(i)
        oy.append(90.0)

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, grid_size, robot_size)

    start_node = GlobalGraphNode(sx, sy)
    goal_node = GlobalGraphNode(gx, gy)

    GlobalGraphEdge(start_node, goal_node, calc_h(goal_node, start_node.x, start_node.y))

    global_graph = {start_node, goal_node}
    path_verified = False
    global_path = []
    while not path_verified:
        if show_animation:
            plt.clf()
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "xr")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

            for node in global_graph:
                for edge in node.getOutEdges():
                    plt.plot([edge.start_node.x, edge.goal_node.x], [edge.start_node.y, edge.goal_node.y],
                             color='green', marker='o', linestyle='dashed', linewidth=2, markersize=6)

        global_path = global_a_start_planning(start_node, goal_node, xw, minx, miny)
        path_verified = True

        if show_animation:
            for edge in global_path:
                plt.plot([edge.start_node.x, edge.goal_node.x], [edge.start_node.y, edge.goal_node.y],
                         color='yellow', marker='x', linestyle='solid', linewidth=2, markersize=6)
            plt.pause(0.001)

        for edge in global_path:

            if edge.checked:
                continue

            global_nodes_to_add, straight_line_solution = global_bug_planning(edge.start_node.x, edge.start_node.y,
                                                                              edge.goal_node.x, edge.goal_node.y,
                                                                              ox, oy, grid_size, robot_size)

            # add all new nodes to the graph
            for new_node in global_nodes_to_add:
                node_to_add = GlobalGraphNode(new_node.x, new_node.y)

                #check if it is already containted in the set
                contained = False
                for old_node in global_graph:
                    if node_to_add.x == old_node.x and node_to_add.y == old_node.y:
                        contained = True
                        break
                if contained:
                    continue

                #add edges to all nodes
                for old_node in global_graph:
                    GlobalGraphEdge(node_to_add, old_node, calc_h(old_node, node_to_add.x, node_to_add.y))
                    GlobalGraphEdge(old_node, node_to_add, calc_h(node_to_add, old_node.x, old_node.y))

                global_graph.add(node_to_add)

            if not straight_line_solution:
                edge.start_node.removeOutConnection(edge)
                edge.goal_node.removeInConnection(edge)
                path_verified = False
                break
            else:
                edge.wasChecked()

    if show_animation:
        plt.clf()
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

        for edge in global_path:
            plt.plot([edge.start_node.x, edge.goal_node.x], [edge.start_node.y, edge.goal_node.y],
                     color='blue', marker='*', linestyle='solid', linewidth=2, markersize=6)
        plt.show()


if __name__ == '__main__':
    main()
