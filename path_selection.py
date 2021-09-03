#!/usr/bin/python3
import sys

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxsize
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.distance < other.distance
        return NotImplemented

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

import heapq

def dijkstra(aGraph, start):
    #print '''Dijkstra's shortest path'''
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)

            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
            #     print 'updated : current = %s next = %s new_dist = %s' \
            #             %(current.get_id(), next.get_id(), next.get_distance())
            # else:
            #     print 'not updated : current = %s next = %s new_dist = %s' \
            #             %(current.get_id(), next.get_id(), next.get_distance())

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)

#for new path add the vertex and edge
def find_path(src, dst):
    g = Graph()
    #Modifiy the below lines for new vertex
    g.add_vertex('h')
    g.add_vertex('a')
    g.add_vertex('b')
    g.add_vertex('c')
    g.add_vertex('d')
    g.add_vertex('e')

    #Always add an edge below if vertex is added
    g.add_edge('h', 'a', 4.2)# 4.2 is the distance from h to a
    g.add_edge('a', 'e', 2.6)
    g.add_edge('e', 'b', 4.2)
    g.add_edge('e', 'd', 3.3)
    g.add_edge('b', 'c', 3.3)
    g.add_edge('c', 'd', 4.2)


    #print 'Graph data:'
    path_list = []
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            #print '( %s , %s, %3d)'  % ( vid, wid, v.get_weight(w))
            path_list.append([vid, wid, v.get_weight(w)])


    dijkstra(g, g.get_vertex(src))

    target = g.get_vertex(dst)
    path = [target.get_id()]
    shortest(target, path)
    #print 'The shortest path : %s' %(path[::-1])

    return path[::-1], path_list


def get_nav(src,dst,heading):#[source, destination, direction from source to destination
    #for New path modify the below list 'headings'
    headings = [
    ['h', 'a', 'N'],
    ['a', 'h', 'S'],
    ['a', 'e', 'W'],
    ['b', 'e', 'N'],
    ['b', 'c', 'W'],
    ['c', 'b', 'E'],
    ['c', 'd', 'N'],
    ['d', 'e', 'E'],
    ['d', 'c', 'S'],
    ['e', 'a', 'E'],
    ['e', 'b', 'S'],
    ['e', 'd', 'W'],
    ]


    path, path_list = find_path(src, dst)
    # for p in headings:
    #     # print(p[0], p[1], p[2])
    #     print(p)
    src = path[0]

    #print(path)

    current_path = []
    for x in path[1:]:
        dest = x
        for paths in path_list:
            if src == paths[0] and dest == paths[1]:
                for direction in headings:
                    if src == direction[0] and dest == direction[1]:
                        #print(src,dest,paths[2],direction[2])
                        current_path.append([src,dest,paths[2],direction[2]])
                        src = x

    #print(current_path)


    turn_matrix = [
    [0, 1, -1, 2],
    [-1, 0, 2, 1],
    [1, 2, 0, -1],
    [2, -1, 1, 0],
    ]



    current_heading = heading
    nav_path = []
    def compute_turn(current_heading, required_heading):
        N, E, W, S = 0, 1, 2, 3
        t = turn_matrix[locals()[current_heading]][locals()[required_heading]]
        if t == -1:
            # print('turn_left()')
            # print('time.sleep(1)')
            nav_path.append('turn_left()')
            nav_path.append('time.sleep(1)')
        elif t == 1:
            # print('turn_right()')
            # print('time.sleep(1)')
            nav_path.append('turn_right()')
            nav_path.append('time.sleep(1)')
        elif t == 2:
            # print('turn_left()')
            # print('time.sleep(1)')
            # print('turn_left()')
            # print('time.sleep(1)')
            nav_path.append('turn_left()')
            nav_path.append('time.sleep(1)')
            nav_path.append('turn_left()')
            nav_path.append('time.sleep(1)')

    #print(path)
    for route in current_path:
        required_heading = route[3]
        #nav_path.append(str(route[0]+route[1]))
        if current_heading == required_heading:
            # print('go_straight('+str(route[2])+')')
            # print('time.sleep(1)')
            nav_path.append(str(route[0]+route[1]))
            nav_path.append('go_straight('+str(route[2])+')')
            nav_path.append('time.sleep(1)')
        else:
            compute_turn(current_heading, required_heading)
            current_heading = required_heading
            # print('go_straight('+str(route[2])+')')
            # print('time.sleep(1)')
            nav_path.append(str(route[0]+route[1]))
            nav_path.append('go_straight('+str(route[2])+')')
            nav_path.append('time.sleep(1)')
    return path, nav_path, current_heading

# if __name__ == '__main__':
#     # print('Hello')
#     # path = get_path('H', 'd')
#     # for p in path:
#     #     print(p)
#     a, b,c = get_nav('a','b','N')
#     print(b)
