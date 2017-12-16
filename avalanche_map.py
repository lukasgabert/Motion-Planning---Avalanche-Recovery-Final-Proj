import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
# from stl import mesh
import random
import graph_search_improved as gsi
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

    def __init__(self, map_path=None, voxel_delta=0.5):
        self.rows    = None
        self.cols    = None
        self.heights = None
        self.beacon_map    = []
        self.seen_beacons  = []
        self.found_beacons = []
        self.search_actions = ['f','b','l','r']
        self.a_map   = None
        self.mesh_map = None
        self.height_map = None

        # stl file members
        self.x_range = None               # stl min to max w/ step size delta: x axis
        self.y_range = None               # stl min to max w/ step size delta: y axis
        self.z_range = None               # stl min to max w/ step size delta: z axis

        self.voxel_delta = voxel_delta    # how accurate we want our cell decomposition

        # waypoints
        self.waypoints = None
        self.sized_waypoints = []
        self.total_path = None

        # for timing
        self.time = 0.0

        if map_path is not None:
            self.read_stl(map_path)

    def read_stl(self, file_path='Maps\\Simple_STL.stl'):
        self.mesh_map = mesh.Mesh.from_file(file_path)

        xmin = int(min([min([x[0] for x in vec]) for vec in self.mesh_map.vectors]))
        xmax = int(max([max([x[0] for x in vec]) for vec in self.mesh_map.vectors]))+1
        ymin = int(min([min([x[1] for x in vec]) for vec in self.mesh_map.vectors]))
        ymax = int(max([max([x[1] for x in vec]) for vec in self.mesh_map.vectors]))+1
        zmin = int(min([min([x[2] for x in vec]) for vec in self.mesh_map.vectors]))
        zmax = int(max([max([x[2] for x in vec]) for vec in self.mesh_map.vectors]))+1

        self.x_range = np.arange(xmin, xmax, self.voxel_delta)
        self.y_range = np.arange(ymin, ymax, self.voxel_delta)
        self.z_range = np.arange(zmin, zmax, self.voxel_delta)

        print 'x_range: {0}\ny_range: {1}\nz_range: {2}'.format(self.x_range, self.y_range, self.z_range)
        print len(self.x_range)
        print len(self.y_range)
        print len(self.z_range)

        dt = np.dtype([('occupied', np.bool)])
        self.a_map = np.zeros((len(self.x_range), len(self.y_range), len(self.z_range)), dtype=np.bool)

        self.voxelize(self.mesh_map, False, False)

    def voxelize(self, mesh_map, draw_voxels=False, draw_mesh=False):
        if draw_mesh:
            fig1 = plt.figure(1)
            ax1 = mplot3d.Axes3D(fig1)

        if draw_voxels:
            fig2 = plt.figure(2)
            ax2 = mplot3d.Axes3D(fig2)

        r, c, h = 0, 0, 0
        for x in self.x_range:
            r = 0
            for y in self.y_range:
                h = 0
                for z in self.z_range:
                    print '({0},{1},{2})'.format(c, r, h)
                    # h += 1
                    # continue
                    B1 = (x, y, z)
                    B2 = (x + self.voxel_delta, y + self.voxel_delta, z + self.voxel_delta)
                    for vec in mesh_map.vectors:
                        v0 = vec[0]
                        v1 = vec[1]
                        v2 = vec[2]

                        if self.check_line_in_box(B1, B2, v0, v1)[0] or \
                           self.check_line_in_box(B1, B2, v1, v2)[0] or \
                           self.check_line_in_box(B1, B2, v2, v0)[0]:
                            self.a_map[c][r][h] = True

                            if draw_voxels:
                                v = np.array([[x, y, z],
                                              [x + self.voxel_delta, y, z],
                                              [x, y + self.voxel_delta, z],
                                              [x, y, z + self.voxel_delta],
                                              [x + self.voxel_delta, y + self.voxel_delta, z],
                                              [x + self.voxel_delta, y, z + self.voxel_delta],
                                              [x, y + self.voxel_delta, z + self.voxel_delta],
                                              [x + self.voxel_delta, y + self.voxel_delta, z + self.voxel_delta]])
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
                                sides = [[v[0], v[1], v[4], v[2]], [v[3], v[5], v[7], v[6]], [v[0], v[1], v[5], v[3]],
                                         [v[2], v[4], v[7], v[6]], [v[0], v[2], v[6], v[3]], [v[1], v[4], v[7], v[5]]]


                                ax2.add_collection3d(mplot3d.art3d.Poly3DCollection(sides,
                                                                                    facecolors='cyan',
                                                                                    linewidths=1,
                                                                                    edgecolors='r',
                                                                                    alpha=.25))

                            break
                    h += 1
                r += 1
            c += 1

        if draw_mesh:
            for vec in mesh_map.vectors:
                v0 = vec[0]
                v1 = vec[1]
                v2 = vec[2]

                xs = [v0[0], v1[0], v2[0], v0[0]]
                ys = [v0[1], v1[1], v2[1], v0[1]]
                zs = [v0[2], v1[2], v2[2], v0[2]]

                ax1.plot(xs, ys, zs, 'b')

            scale = mesh_map.points.flatten(-1)
            ax1.auto_scale_xyz(scale, scale, scale)
            fig1.show()
            fig1.savefig('gentle-slope-wireframe_0_5.png')

        if draw_voxels:
            scale = mesh_map.points.flatten(-1)
            ax2.auto_scale_xyz(scale, scale, scale)
            fig2.show()
            fig2.savefig('gentle-slope-voxel_0_5.png')

        self.rows = r
        self.cols = c
        self.heights = h

        self.draw_map()
        self.print_map()
        self.fill_in_terrain()
        self.print_map()
        self.draw_map()
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

    def fill_in_terrain(self):
        for c in xrange(self.cols):
            for r in xrange(self.rows):
                h = self.get_highest_z(r, c)
                if h == -1:
                    h = self.get_average_neighbor_height(r, c)
                    self.a_map[c][r][h] = True
                for h_prime in range(0, h):
                    self.a_map[c][r][h_prime] = True

    def get_average_neighbor_height(self, r, c):
        h_array = []
        for x in range(c-1, c+2, 1):
            for y in range(r-1, r+2, 1):
                try:
                    h = self.get_highest_z(y, x)
                    if h >= 0:
                        h_array.append(h)
                except:
                    pass

        h = int(sum(h_array)/len(h_array))
        return h

    def get_highest_z(self, r, c):
        for h in range(self.heights-1, -1, -1):
            if self.a_map[c][r][h]:
                return h
        return -1

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

    def draw_map(self):
        fig = plt.figure()
        ax = mplot3d.Axes3D(fig)

        for c in xrange(self.cols):
            for r in xrange(self.rows):
                for h in xrange(self.heights):
                    if self.a_map[c][r][h]:
                        v = np.array([[c, r, h],
                                      [c+1, r, h],
                                      [c, r+1, h],
                                      [c, r, h+1],
                                      [c+1, r+1, h],
                                      [c+1, r, h+1],
                                      [c, r+1, h+1],
                                      [c+1, r+1, h+1]])
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
                        sides = [[v[0], v[1], v[4], v[2]], [v[3], v[5], v[7], v[6]], [v[0], v[1], v[5], v[3]],
                                 [v[2], v[4], v[7], v[6]], [v[0], v[2], v[6], v[3]], [v[1], v[4], v[7], v[5]]]

                        ax.add_collection3d(mplot3d.art3d.Poly3DCollection(sides,
                                                                           facecolors='cyan',
                                                                           linewidths=1,
                                                                           edgecolors='r',
                                                                           alpha=.25))

        ax.auto_scale_xyz([-5,self.cols+5], [-5, self.rows+5], [-5, self.heights+5])
        plt.show()

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

    def get_height_map(self, num_averaging=4):
        new_cols = int(self.cols/num_averaging)+1
        new_rows = int(self.rows/num_averaging)+1
        old_rows = self.rows
        old_cols = self.cols

        self.height_map = np.zeros((new_cols, new_rows), np.float64)
        for c in xrange(new_cols):
            x_c = num_averaging*c
            for r in xrange(new_rows):
                y_r = num_averaging*r
                h_avg = 0.0
                count = 0
                for x in range(x_c, x_c+num_averaging+1, 1):
                    for y in range(y_r, y_r+num_averaging+1, 1):
                        try:
                            h = self.get_highest_z(y, x)
                            h_avg += h
                            count += 1
                        except:
                            pass
                h_avg /= count
                self.height_map[c][r] = h_avg
        print self.height_map

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
        if s_new[0] >= self.cols:
            s_new[0] = self.cols-1
        if s_new[1] >= self.rows:
            s_new[1] = self.rows-1
        if s_new[2] >= self.heights:
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
        #print minimize_visited_states[-2]
        minimize_visited_states.append(minimize_visited_states[-2])
        s = minimize_visited_states[-1]
        #print 'cur_action: ',cur_action
        #print 'dist2beacon(after 1st minimization): ',dist2beacon
        state_options = []
        for action in self.search_actions:
            possible_state = self.transfer_func(s,action)
            possible_dist2next = self.beacon_map[possible_state[0]][possible_state[1]][possible_state[2]][active_beacon_index][1]
            state_options.append([possible_state, action, possible_dist2next])
            #print 'current state:', current_state
            #print options
            d_min = 10000
            for d in state_options:
                d_choice = d[2]
                if d_choice <= d_min:
                    d_min = d_choice
                    s = d[0]
                    cur_action = d[1]
                    dist2beacon = d[2]

        #minimize in the second direction
        s,minimize_visited_states2,dist2beacon = self.line_search(s,cur_action,dist2beacon,active_beacon_index)
        print 'minimize_visited_states2: ', minimize_visited_states2
        #print minimize_visited_states2
        minimize_visited_states2.append(minimize_visited_states2[-2])
        s = minimize_visited_states2[-1]
        visited_states = minimize_visited_states + minimize_visited_states2
        self.found_beacons.append(active_beacon_index)
        #print 'total path: ', visited_states
        return s,visited_states

    def get_coverage_waypoints(self, cliff_height):
            g = gsi.GridMap(self.height_map, cliff_height)

            self.waypoints = gsi.path_coverage(g, g.init_pos, g.transition, gsi._ACTIONS)
            # g.display_map_new(self.waypoints)

            # g.display_cell_values()
            # g.display_cell_height()

    def follow_coverage_waypoints(self, num_averaging):
        # From max of rows, cols, go to first waypoint.
        for i, point in enumerate(self.waypoints):
            self.sized_waypoints.insert(0, (point[0]*(num_averaging), point[1]*(num_averaging)))
        # print self.waypoints
        # print self.sized_waypoints
        current_loc = (self.cols-1, self.rows-1)
        current_loc_z = self.get_highest_z(current_loc[1], current_loc[0])
        new_loc_z = 0.0
        list_of_loc = []
        list_of_loc.append(current_loc)
        while len(self.sized_waypoints) > 0:
            next_point = self.sized_waypoints.pop()
            while current_loc != next_point:
                # go whatever direction is needed
                # print current_loc
                # print next_point
                if current_loc[0] < next_point[0]:
                    current_loc = (current_loc[0] + 1, current_loc[1])
                    self.time += 1
                elif current_loc[0] > next_point[0]:
                    current_loc = (current_loc[0] - 1, current_loc[1])
                    self.time += 1
                elif current_loc[1] < next_point[1]:
                    current_loc = (current_loc[0], current_loc[1] + 1)
                    self.time += 1
                elif current_loc[1] > next_point[1]:
                    current_loc = (current_loc[0], current_loc[1] - 1)
                    self.time += 1
                # DO HEIGHT CHECKING
                new_loc_z = self.get_highest_z(current_loc[1], current_loc[0])
                z_diff = current_loc_z - new_loc_z
                self.time += abs(z_diff)
                current_loc_z = new_loc_z
                list_of_loc.append(current_loc)
                #print current_loc[1]
                #print current_loc[0]
                s = [current_loc[0], current_loc[1], current_loc_z]
                #search self.beacon map at state "s" for potential beacons.
                beacon_index = 0
                for beacon_list in self.beacon_map[s[0]][s[1]][s[2]]:
                    if beacon_list[0]:
                        active_beacon_index = beacon_index
                        #see if I've already found it
                        if  active_beacon_index not in self.found_beacons:
                            state_options = []
                            dist2beacon = self.beacon_map[s[0]][s[1]][s[2]]
                            for action in self.search_actions:
                                possible_state = self.transfer_func(s,action)
                                possible_dist2next = self.beacon_map[possible_state[0]][possible_state[1]][possible_state[2]][active_beacon_index][1]
                                state_options.append([possible_state, action, possible_dist2next])
                            #print 'current state:', current_state
                            #print options
                            d_min = 10000
                            for d in state_options:
                                d_choice = d[2]
                                if d_choice <= d_min:
                                    d_min = d_choice
                                    #s = d[0]
                                    best_action = d[1]
                                    #dist2beacon = d[2]
                            current_state,path = self.beacon_search(s,best_action, dist2beacon,active_beacon_index)
                            list_of_loc.append(path)
                            self.time += len(path)
                            #returning to the point where we left the local planner
                            list_of_loc.append(current_loc)
                    beacon_index += 1
        # print "rows", self.rows
        # print "cols", self.cols
        print "time", self.time
        self.total_path = list_of_loc
        print 'length of total path: ', len(self.total_path)
        print 'total path length: ', self.total_path



    def display_map_new(self, path=[], visited={}):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        path = self.total_path

        _GOAL_COLOR = 0.75
        _INIT_COLOR = 0.25
        _PATH_COLOR_RANGE = _GOAL_COLOR - _INIT_COLOR
        _VISITED_COLOR = 0.9

        plt.axis([0-.5, self.rows-.5, self.cols-.5, 0-.5])
        display_grid = np.array(np.zeros((self.cols, self.rows ), dtype=np.float32), dtype=np.float32)
        plt.hold(True)

        # Color all visited nodes if requested
        # for v in visited:
        #    display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        for i, p in enumerate(path):
            disp_col_forlines = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
            display_grid[p] = disp_col_forlines
            if i > 0:
                # print p
                # print path[i-2]
                plt.plot([path[i-1][1], p[1]], [path[i-1][0], p[0]])  # , color)
                # plt.pause(0.001)

        # display_grid[self.init_pos] = _INIT_COLOR
        # display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plt.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plt.show()
