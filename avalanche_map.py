import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh
import random
import copy


'''
class avalanche_map contains a map of the terrain
Assumptions:
    1. The dimensions of the map are scaled appropriately (when reading the stl file an x-axis range from -3 to 3 could
        equate to 3 miles of landscape starting at some specified point. The quadcopter would have algorithms developed
        inside it to account for this scaling and offset when controlling its position and speed).
    2. The map is a section of the mountain that has been obtained via a third party software as a wire frame input. We
        will quickly voxelize the input and transform it into a reasonable coordinate system for the UAV.
'''
class AvalancheMap:

    def __init__(self, map_path=None, voxel_delta=1):
        self.rows          = None
        self.cols          = None
        self.heights       = None
        self.beacon_map    = []
        self.seen_beacons  = []
        self.found_beacons = []
        self.a_map         = None
        self.search_actions = ['f','b','l','r']

        # stl file members
        self.x_range = None               # stl min to max w/ step size delta: x axis
        self.y_range = None               # stl min to max w/ step size delta: y axis
        self.z_range = None               # stl min to max w/ step size delta: z axis

        self.voxel_delta = voxel_delta    # how accurate we want our cell decomposition

        if map_path is not None:
            self.read_stl(map_path)

    def read_stl(self, file_path='Maps\\Simple_STL.stl'):
        mesh_map = mesh.Mesh.from_file(file_path)

        xmin = int(min([min([x[0] for x in vec]) for vec in mesh_map.vectors]))
        xmax = int(max([max([x[0] for x in vec]) for vec in mesh_map.vectors]))+1
        ymin = int(min([min([x[1] for x in vec]) for vec in mesh_map.vectors]))
        ymax = int(max([max([x[1] for x in vec]) for vec in mesh_map.vectors]))+1
        zmin = int(min([min([x[2] for x in vec]) for vec in mesh_map.vectors]))
        zmax = int(max([max([x[2] for x in vec]) for vec in mesh_map.vectors]))+1

        self.x_range = np.arange(xmin, xmax, self.voxel_delta)
        self.y_range = np.arange(ymin, ymax, self.voxel_delta)
        self.z_range = np.arange(zmin, zmax, self.voxel_delta)

        dt = np.dtype([('occupied', np.bool)])
        self.a_map = np.zeros((len(self.x_range), len(self.y_range), len(self.z_range)), dtype=np.bool)

        self.voxelize(mesh_map)

    def voxelize(self, mesh_map):
        # fig = plt.figure(1)
        # ax = mplot3d.Axes3D(fig)
        r, c, h = 0, 0, 0

        c = 0
        for x in self.x_range:
            r = 0
            for y in self.y_range:
                h = 0
                for z in self.z_range:
                    B1 = (x, y, z)
                    B2 = (x + self.voxel_delta, y + self.voxel_delta, z + self.voxel_delta)
                    for vec in mesh_map.vectors:
                        v0 = vec[0]
                        v1 = vec[1]
                        v2 = vec[2]
                        #
                        # xs = [v0[0], v1[0], v2[0], v0[0]]
                        # ys = [v0[1], v1[1], v2[1], v0[1]]
                        # zs = [v0[2], v1[2], v2[2], v0[2]]
                        #
                        # ax.plot(xs, ys, zs, 'b')
                        if self.check_line_in_box(B1, B2, v0, v1)[0] or \
                           self.check_line_in_box(B1, B2, v1, v2)[0] or \
                           self.check_line_in_box(B1, B2, v2, v0)[0]:
                            self.a_map[c][r][h] = True
                            break

                            # v = np.array([[x, y, z],
                            #               [x + self.voxel_delta, y, z],
                            #               [x, y + self.voxel_delta, z],
                            #               [x, y, z + self.voxel_delta],
                            #               [x + self.voxel_delta, y + self.voxel_delta, z],
                            #               [x + self.voxel_delta, y, z + self.voxel_delta],
                            #               [x, y + self.voxel_delta, z + self.voxel_delta],
                            #               [x + self.voxel_delta, y + self.voxel_delta, z + self.voxel_delta]])
                            #
                            #       v6       v7
                            #       +--------+
                            #      /|       /|
                            #  v3 / |   v5 / |
                            #    +--------+  |
                            #    |  +-----|--+
                            #    | / v2   | / v4
                            #    |/       |/
                            #    +--------+
                            #    v0       v1
                            #
                            # bottom, top, front, back, left, right
                            # sides = [[v[0], v[1], v[4], v[2]], [v[3], v[5], v[7], v[6]], [v[0], v[1], v[5], v[3]],
                            #          [v[2], v[4], v[7], v[6]], [v[0], v[2], v[6], v[3]], [v[1], v[4], v[7], v[5]]]
                            #
                            #
                            # ax.add_collection3d(mplot3d.art3d.Poly3DCollection(sides,
                            #                                                    facecolors='cyan',
                            #                                                    linewidths=1,
                            #                                                    edgecolors='r',
                            #                                                    alpha=.25))
                    h += 1
                r += 1
            c += 1
        # for vec in mesh_map.vectors:
        #     v0 = vec[0]
        #     v1 = vec[1]
        #     v2 = vec[2]
        #
        #     xs = [v0[0], v1[0], v2[0], v0[0]]
        #     ys = [v0[1], v1[1], v2[1], v0[1]]
        #     zs = [v0[2], v1[2], v2[2], v0[2]]
        #
        #     ax.plot(xs, ys, zs, 'b')

        # ax.plot(v0[0], v0[1], v0[2])
        # ax.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh_map.vectors[0]))

        # scale = mesh_map.points.flatten(-1)
        # ax.auto_scale_xyz(scale, scale, scale)

        # fig2 = plt.figure(2)
        # ax2 = mplot3d.Axes3D(fig2)
        #
        # self.rows    = r
        # self.cols    = c
        # self.heights = h
        #
        # for c in range(self.cols):
        #     for r in range(self.rows):
        #         for h in range(self.heights):
        #             if self.a_map[c][r][h]:
        #                 ax2.scatter(c, r, h)
        #
        # plt.show()

        self.rows = r
        self.cols = c
        self.heights = h

        self.print_map()
        self.fill_in_terrain()
        self.print_map()
#
#       +--------+
#      /|       /|
#     / |      / |
#    +--------+  |
#    |  +-----|--+
#    | /      | /
#    |/       |/
#    +--------+
#  Pcube
#
    def check_line_in_box(self, B1, B2, L1, L2):
        if L2[0] < B1[0] and L1[0] < B1[0]:
            return False, None

        if L2[0] > B2[0] and L1[0] > B2[0]:
            return False, None

        if L2[1] < B1[1] and L1[1] < B1[1]:
            return False, None

        if L2[1] > B2[1] and L1[1] > B2[1]:
            return False, None

        if L2[2] < B1[2] and L1[2] < B1[2]:
            return False, None

        if L2[2] > B2[2] and L1[2] > B2[2]:
            return False, None

        if B1[0] < L1[0] < B2[0] and \
           B1[1] < L1[1] < B2[1] and \
           B1[2] < L1[2] < B2[2]:
            return True, L1

        tests = [(L1[0] - B1[0], L2[0] - B1[0], L1, L2, 0),
                 (L1[1] - B1[1], L2[1] - B1[1], L1, L2, 1),
                 (L1[2] - B1[2], L2[2] - B1[2], L1, L2, 2),
                 (L1[0] - B2[0], L2[0] - B2[0], L1, L2, 0),
                 (L1[1] - B2[1], L2[1] - B2[1], L1, L2, 1),
                 (L1[2] - B2[2], L2[2] - B2[2], L1, L2, 2)]

        for test in tests:
            (intersects, hit) = self.get_intersection(test[0], test[1], test[2], test[3])
            if not intersects:
                continue

            inside = self.in_box(hit, B1, B2, test[4])

            if inside:
                return True, hit

        return False, None

    @staticmethod
    def get_intersection(fDst1, fDst2, P1, P2):
        if (fDst1 * fDst2) >= 0.0:
            return False, None

        if fDst1 == fDst2:
            return False, None

        hit = P1 + (P2-P1) * (-fDst1 / (fDst2-fDst1))
        return True, hit

    @staticmethod
    def in_box(hit, B1, B2, axis):
        if axis == 0 and B1[2] < hit[2] < B2[2] and B1[1] < hit[1] < B2[1]:
            return True

        if axis == 1 and B1[2] < hit[2] < B2[2] and B1[0] < hit[0] < B2[0]:
            return True

        if axis == 2 and B1[1] < hit[1] < B2[1] and B1[0] < hit[0] < B2[0]:
            return True

        return False

    def print_map(self):
        xy_planes = {}

        for h in xrange(self.heights):
            xy_plane = np.zeros((self.cols, self.rows), np.bool)

            for c in xrange(self.cols):
                for r in xrange(self.rows):
                    xy_plane[c][r] = self.a_map[c][r][h]

            xy_planes[h] = xy_plane[:]

        for h in xy_planes.keys():
            print xy_planes[h]

    def fill_in_terrain(self):
        for c in xrange(self.cols):
            for r in xrange(self.rows):
                for h in range(self.heights-1, -1, -1):
                    if self.a_map[c][r][h]:
                        for h_prime in range(0, h):
                            self.a_map[c][r][h_prime] = True
                        break

    def save_map(self, file_name=None):
        if file_name is None:
            map_file = file('Maps\\default_map.txt', 'w')
        else:
            map_file = file(file_name, 'w')

        # Save row, column, height dimensions of map
        map_file.write('rows:    {0}\n'.format(self.rows))
        map_file.write('columns: {0}\n'.format(self.cols))
        map_file.write('heights: {0}\n'.format(self.heights))

        # save data
        for h in xrange(self.heights):
            for r in xrange(self.rows-1, -1, -1):
                for c in xrange(self.cols):
                    if self.a_map[c][r][h]:
                        map_file.write('1 ')
                    else:
                        map_file.write('0 ')
                map_file.write('\n')
            map_file.write('\n')

        map_file.close()

    def load_map(self, file_name=None):
        if file_name is None:
            map_file = file('Maps\\default_map.txt', 'r')
        else:
            map_file = file(file_name, 'r')

        # Get row, column, height dimensions of map
        line = map_file.readline()
        self.rows = int(line.split()[1])

        line = map_file.readline()
        self.cols = int(line.split()[1])

        line = map_file.readline()
        self.heights = int(line.split()[1])

        # retrieve data
        self.a_map = np.zeros((self.cols, self.rows, self.heights), np.bool)
        for h in xrange(self.heights):
            for r in xrange(self.rows-1, -1, -1):
                line = map_file.readline().split()
                for c in xrange(self.cols):
                    if line[c] == '1':
                        self.a_map[c][r][h] = True
                    else:
                        self.a_map[c][r][h] = False
            line = map_file.readline()

        map_file.close()

    def beacon_location(self, num_beacons, avalanche_size='small', default_beacon_range = 60):
        ''' Inputs:
            num_beacons (int) = Number of people buried
            avalanche_size (str) = Classification for how large the avalanche is. Options: small, medium, large
            default_beacon_range (int) = Ideal case beacon transmission range measured in feet (ft)

            Returns:
            the x,y,z coordinate of the beacon as well as the transmission range [x,y,depth, range]
        '''

        beacon_vector = []
        max_x = self.cols
        max_y = self.rows
        min_depth = 1 #feet
        #determining the potential depth
        if avalanche_size == 'small':
            max_depth = 3
        elif avalanche_size == 'medium':
            max_depth = 6
        elif avalanche_size == 'large':
            max_depth = 13
        else:
            print 'invalid avalanche size classification, make sure you spelled the size correctly'
            return None

        #print 'max_depth', max_depth
        for i in xrange(num_beacons):
            #find random x and y location for beacon
            rand_x = int(random.random()*max_x)
            rand_y = int(random.random()*max_y)
            #need to change this to look at the x/y coordinate column and find the ground level node.
            depth = int(random.random()*(max_depth-min_depth)+min_depth)
            #print 'beacon depth = ',depth
            i = 0
            for z in self.a_map[rand_x][rand_y]:
                height = i
                #print rand_x,rand_y,i,z
                if not z:
                    beacon_height = i-depth
                    if beacon_height < 0:
                        beacon_height = 0
                    break
                i += 1
            #scaling the range of the beacon based on how deep it has been buried
            if depth >= 6:
                beacon_range = float(default_beacon_range)*0.4
            elif depth < 6 and depth >= 4:
                beacon_range = float(default_beacon_range)*0.8
            elif depth < 4:
                beacon_range = float(default_beacon_range)*0.9
            beacon_state = [rand_x, rand_y, beacon_height, beacon_range]
            beacon_vector.append(beacon_state)
        return beacon_vector

    def distance_calculator(self, beacon_array):
        ''' Inputs:
            beacon_array (array): contains the following information: [x,y,z,transmission range] for each beacon
            Returns:
            beacon_map (array of tuples) : is the same size as the grid map. Each entry in the 3D grid_map vector contains an
            array of length (# of beacons) containing tuples. The first value is a flag notifying if the beacon can be seen,
            the second value is the distance to the beacon.
        '''
        #create the beacon map
        for x in xrange(self.cols):
            row_vector = []
            for y in xrange(self.rows):
                height_vector = []
                for z in xrange(self.heights):
                    #print x,y,z
                    beacon_visibility_array = []
                    for beacon in beacon_array:
                        distance_to_beacon = ((x-beacon[0])**2+(y-beacon[1])**2+(z-beacon[2])**2)**0.5
                        if distance_to_beacon <= beacon[3]:
                            visable = True
                        else:
                            visable = False
                        beacon_visibility_array.append((visable,distance_to_beacon))
                    height_vector.append(beacon_visibility_array)
                row_vector.append(height_vector)
            self.beacon_map.append(row_vector)
        return None

    def plot_beacon_grid(self, beacon_location_vector):
        '''plots the beacon_map and the grid_map'''

        x_max = self.cols
        y_max = self.rows
        z_max = self.heights
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection='3d')

        #plotting the beacons
        for beacon in beacon_location_vector:
            x = beacon[0]
            y = beacon[1]
            z = beacon[2]
            ax1.scatter(x,y,z,c='b',marker='*')
        ax1.set_ylim(0,y_max)
        ax1.set_xlim(0,x_max)
        ax1.set_zlim(0,z_max)
        ax1.set_xlabel('X_Label')
        ax1.set_ylabel('Y_Label')
        ax1.set_zlabel('Z_Label')
        plt.show()

        #plotting beacon map and the grid_map together
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111,projection='3d')
        for x in xrange(self.cols):
            for y in xrange(self.rows):
                for z in xrange(self.heights):
                    i = 0
                    for sight_flag in self.beacon_map [x][y][z]:
                        if sight_flag[0]:
                            #print sight_flag
                            #print self.a_map[x][y][z]
                            if i == 0:
                                c = 'r'
                            elif i == 1:
                                c = 'g'
                            elif i == 2:
                                c = 'b'
                            elif i == 3:
                                c = 'y'
                            else:
                                c = 'k'
                            ax2.scatter(x, y, z, c=c, marker='o')
                        i += 1
                    if self.a_map[x][y][z]:
                        ax2.scatter(x, y, z, c='k', marker = 's')
        ax2.set_ylim(0,y_max)
        ax2.set_xlim(0,x_max)
        ax2.set_zlim(0,z_max)
        ax2.set_xlabel('X_Distance')
        ax2.set_ylabel('Y_Distance')
        ax2.set_zlabel('Z_Height')
        plt.show()

    def opposite_action (self,a):
        if a == 'u':
            a_opposite = 'd'
        elif a == 'd':
            a_opposite = 'u'
        elif a == 'r':
            a_opposite = 'l'
        elif a == 'l':
            a_opposite = 'r'
        elif a == 'f':
            a_opposite = 'b'
        elif a == 'b':
            a_opposite = 'f'
        else:
            a_opposite = None
        return a_opposite

    def perpendicular_action (self,a):
        if a == 'f':
            a_perpendicular = 'r'
        elif a == 'r':
            a_perpendicular = 'b'
        elif a == 'b':
            a_perpendicular = 'l'
        elif a == 'l':
            a_perpendicular = 'u'
        else:
            a_perpendicular = None
        return a_perpendicular

    def transfer_func(self,s,a):
        s_new = copy.deepcopy(s)
        if a == 'u':    #up
            s_new[2] += 1
        elif a == 'd':  #down
            s_new[2] -= 1
        elif a == 'f':  #forward
            s_new[1] += 1
        elif a == 'b':  #backwards
            s_new[1] -= 1
        elif a == 'r':  #right
            s_new[0] += 1
        elif a == 'l':  #left
            s_new[0] -= 1
        elif a == 'ul': #up-left
            s_new[0] -= 1
            s_new[2] += 1
        elif a == 'ur': #up-right
            s_new[0] += 1
            s_new[2] += 1
        elif a == 'uf': #up-forward
            s_new[1] += 1
            s_new[2] += 1
        elif a == 'ub': #ub-back
            s_new[1] -= 1
            s_new[2] += 1
        elif a == 'df': #down-forward
            s_new[1] += 1
            s_new[2] -= 1
        elif a == 'db': #down-back
            s_new[2] -= 1
            s_new[1] -= 1
        elif a == 'dl': #down-left
            s_new[0] -= 1
            s_new[2] -= 1
        elif a == 'dr': #down-right
            s_new[0] += 1
            s_new[2] -= 1
        elif a == 'fl': #forward-left
            s_new[0] -= 1
            s_new[1] += 1
        elif a == 'fr': #forward-right
            s_new[0] += 1
            s_new[1] += 1
        elif a == 'bl': #back-left
            s_new[0] -= 1
            s_new[1] -= 1
        elif a == 'br': #back-right
            s_new[0] += 1
            s_new[1] -= 1
        else:
            print 'invalid action'

        #reset values that are outside of the map limits
        if s_new[0] > self.cols:
            s_new[0] = self.cols-1
        if s_new[1] > self.rows:
            s_new[1] = self.rows-1
        if s_new[2] > self.heights:
            s_new[2] = self.heights-1
        if s_new[0] < 0:
            s_new[0] = 0
        if s_new[1] < 0:
            s_new[1] = 0
        if s_new[2] < 0:
            s_new[2] = 0
        return s_new

    def line_search(self,s,cur_action,dist2beacon,active_beacon_index):
        visited_states = []
        s_prime = self.transfer_func(s, cur_action) #move to new step
        visited_states.append(s_prime)
        new_dist2beacon = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][0][1] #new distance to beacon
        beacon_visibility = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][0] #checking beacon signal
        while new_dist2beacon < dist2beacon and beacon_visibility:
            dist2beacon = new_dist2beacon   #reassign minimum distance for comparison
            s = s_prime #new current state
            #take next step
            s_prime = self.transfer_func(s, cur_action)
            collision = self.a_map[s_prime[0]][s_prime[1]][s_prime[2]]  #check to see if s_prime in collision
            # if s_prime is in collision, step up and keep trying to move laterally
            if collision:   #s_prime is in collision
                while collision and beacon_visibility:
                    #step up
                    action = 'u'
                    s = self.transfer_func(s,action)
                    temp_distance = self.beacon_map[s[0]][s[1]][s[2]][active_beacon_index][1]
                    beacon_visibility = self.beacon_map[s[0]][s[1]][s[2]][active_beacon_index][0]
                    if not beacon_visibility:
                        break
                    visited_states.append(s)
                    #try to step forward
                    s_prime = self.transfer_func(s,cur_action)
                    collision = self.a_map[s_prime[0]][s_prime[1]][s_prime[2]]
                    if collision:
                        continue
                    else:
                        new_dist2beacon = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][1]
                        if temp_distance < new_dist2beacon: #if spot before I went up was the minimum, then go back to that spot and exit.
                            s_prime = s
                            return s_prime,visited_states,temp_distance
                        else:
                            visited_states.append(s_prime) #add the s_prime to the visited array and continue on
                            beacon_visibility = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][0]
            else:
                beacon_visibility = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][0]
                if beacon_visibility:
                    new_dist2beacon = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][1]
                    visited_states.append(s_prime)
        #if the beacon is lost due to an action, return to the state where you could see the beacon and declare
        #this as the min point and exit
        if not beacon_visibility:
            s_prime = visited_states[-1]
        return s_prime,visited_states, new_dist2beacon

    def beacon_search(self,s,cur_action,dist2beacon,active_beacon_index):
        '''  Inputs:
            s = state that the drone is at when it first found the beacon
            cur_action = action the drone just took to arrive at the state where it could see the beacon
            active_beacon_index = Index of the beacon. the drone will keep searching this index in the beacon grid to make sure
            the drone maintains connectivity to the antenna
            dist2beacon = distance to the beacon from the cur_state
        Returns:
        '''
        visited_states = []
        if active_beacon_index in self.found_beacons:
            return s,visited_states
        if active_beacon_index not in self.seen_beacons:
            self.seen_beacons.append(active_beacon_index)

        #minimize in the first direction
        s,minimize_visited_states,dist2beacon = self.line_search(s,cur_action,dist2beacon,active_beacon_index)
        #print 'minimize_visited_states: ', minimize_visited_states
        #print 'cur_action: ',cur_action
        #print 'dist2beacon(after 1st minimization): ',dist2beacon

        #determine perpendicular direction to move
        a_perpendicular = self.perpendicular_action(cur_action)
        #print 'a_perpendicular: ',a_perpendicular
        s_prime = self.transfer_func(s, a_perpendicular)
        minimize_visited_states.append(s_prime)
        temp_distance = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][1]
        #print 'temp_distance: ',temp_distance
        #check if that was the right distance
        if temp_distance >= dist2beacon:
            #print 'perpendicular distance is wrong'
            a_opposite = self.opposite_action(a_perpendicular)
            #print 'a_opposite: ', a_opposite
            minimize_visited_states.append(s)
            s_prime = self.transfer_func(s, a_opposite)
            dist2beacon = self.beacon_map[s_prime[0]][s_prime[1]][s_prime[2]][active_beacon_index][1]
            #print 'dist2beacon: ', dist2beacon
            minimize_visited_states.append(s_prime)
            s = s_prime
            cur_action = a_opposite
        else:
            #print 'perpendicular distance is correct'
            cur_action = a_perpendicular
            dist2beacon = temp_distance
        #print 'cur_action: ', cur_action
        #print 'dist2beacon(going into next line search): ',dist2beacon

        #minimize in the second direction
        s,minimize_visited_states2,dist2beacon = self.line_search(s,cur_action,dist2beacon,active_beacon_index)
        #print 'minimize_visited_states2: ', minimize_visited_states2
        visited_states = minimize_visited_states+minimize_visited_states2
        self.found_beacons.append(active_beacon_index)
        #print 'total path: ', visited_states
        return s,visited_states

    def plot_master_path(self,beacon_locations,visited_list):

        x_max = self.cols
        y_max = self.rows
        z_max = self.heights
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection='3d')
        #plotting the beacons
        for beacon in beacon_locations:
            x = beacon[0]
            y = beacon[1]
            z = beacon[2]
            ax1.scatter(x,y,z,c='b',marker='*')
        #plotting the path
        #print visited_list [0]
        for states in visited_list:
            x = states[0]
            y = states[1]
            z = states[2]
            #print x,y,z
            ax1.scatter(x,y,z,c='r',marker='o')
        ax1.set_ylim(0,y_max)
        ax1.set_xlim(0,x_max)
        ax1.set_zlim(0,z_max)
        ax1.set_xlabel('X_Label')
        ax1.set_ylabel('Y_Label')
        ax1.set_zlabel('Z_Label')
        plt.show()
