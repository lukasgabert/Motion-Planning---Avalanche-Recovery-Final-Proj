import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plotter

def beacon_location(max_x,max_y,num_beacons,avalanche_size = 'small',default_beacon_range=15):
    beacon_vector = []
    min_depth = 1 #feet
    max_depth = 5 #place_holder
    #determining the potential depth
    if avalanche_size == 'small':
        max_depth = 3
    elif avalanche_size == 'medium':
        max_depth = 6
    elif avalanche_size == 'large':
        max_depth = 13
    else:
        print 'invalid avalanche_size classification, make sure you spelled the size correctly'
        return None

    print 'max_depth',max_depth
    for i in xrange(num_beacons):
        #find random x and y location for beacon
        rand_x = random.random()*max_x
        rand_y = random.random()*max_y
        depth = random.random()*(max_depth-min_depth)+min_depth
        if depth >= 6:
            beacon_range = default_beacon_range*0.5
        elif depth < 6 and depth >= 4:
            beacon_range = default_beacon_range*0.8
        elif depth < 4:
            beacon_range = default_beacon_range
        beacon_state = [rand_x,rand_y,depth,beacon_range]
        beacon_vector.append(beacon_state)
    return beacon_vector

def distance_calculator(beacon_array,state_space):
    beacon_map = []
    for state in state_space:
        beacon_distance_array = []
        for beacon_state in beacon_array:
            #print beacon_state
            #print state
            distance = ((state[0]-beacon_state[0])**2+(state[1]-beacon_state[1])**2+(state[2]-beacon_state[2])**2)**0.5
            if distance < beacon_state[3]:
                see_beacon = True
                #print 'state',state
                #print 'distance',distance
                #print 'beacon range',beacon_state[3]
                meh = 1
            else:
                see_beacon = False
            beacon_distance_array.append((state,distance,see_beacon))
        beacon_map.append(beacon_distance_array)
        meh = 1
    return beacon_map

def plot_beacon_grid(beacon_location_vector,beacon_map,x_max,y_max,z_max):
    fig1 = plotter.figure()
    ax1 = fig1.add_subplot(111, projection='3d')

    for beacon in beacon_location_vector:
        x = beacon[0]
        y = beacon[1]
        z = beacon[2]
        ax1.scatter(x,y,z,c='b',marker='^')
    ax1.set_ylim(0,y_max)
    ax1.set_xlim(0,x_max)
    ax1.set_zlim(0,z_max)
    ax1.set_xlabel('X_Label')
    ax1.set_ylabel('Y_Label')
    ax1.set_zlabel('Z_Label')
    plotter.show()

    #plotting graph
    fig2 = plotter.figure()
    ax2 = fig2.add_subplot(111,projection='3d')
    for states in beacon_map:
        #print states
        i = 0
        while i < len(states):
            #print 'current_state_beacon', states[i]
            if states[i][2]:
                x = states[i][0][0]
                y = states[i][0][1]
                z = states[i][0][2]
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
                ax2.scatter(x,y,z,c=c,marker = 'o')
            i += 1

    ax2.set_ylim(0,y_max)
    ax2.set_xlim(0,x_max)
    ax2.set_zlim(0,z_max)
    ax2.set_xlabel('X_Label')
    ax2.set_ylabel('Y_Label')
    ax2.set_zlabel('Z_Label')
    plotter.show()


