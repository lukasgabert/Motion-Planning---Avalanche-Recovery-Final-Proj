import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh
import random



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
        self.rows    = None
        self.cols    = None
        self.heights = None

        self.a_map   = None

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
        beacon_map = []
        for x in xrange(self.cols):
            row_vector = []
            for y in xrange(self.rows):
                height_vector = []
                for z in xrange(self.heights):
                    #print x,y,z
                    beacon_visibility_array = []
                    for beacon in beacon_array:
                        distance_to_beacon = ((x-beacon[0])**2+(y-beacon[1])**2+(z-beacon[2])**2)**0.5
                        meh = 1
                        if distance_to_beacon <= beacon[3]:
                            visable = True
                        else:
                            visable = False
                        beacon_visibility_array.append((visable,distance_to_beacon))
                    height_vector.append(beacon_visibility_array)
                row_vector.append(height_vector)
            beacon_map.append(row_vector)
        return beacon_map

    def plot_beacon_grid(self, beacon_location_vector, beacon_map):
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
                    for sight_flag in beacon_map [x][y][z]:
                        if sight_flag[0]:
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
