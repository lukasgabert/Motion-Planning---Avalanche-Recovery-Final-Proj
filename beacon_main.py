import beacon_setup
import math

#def run_beacon_setup(max_y,max_x,num_beacons,avalanche_size):
#    beacon_location_vector = beacon_setup.beacon_location(max_x,max_y,num_beacons,avalanche_size)
#    return beacon_location_vector

if __name__ == '__main__':

    num_beacons = 5
    y_max = 40  #feet
    x_max = 60  #feet
    z_max = 20  #feet
    slope = 10 #degrees
    default_beacon_range = 15
    state_space = []

    #setting up state space
    for z in xrange(z_max):
        for y in xrange(y_max):
            for x in xrange(x_max):
                #calculate if the beacon is buried (below 10 degree slope)
                state_space.append([x,y,z])
    avalanche_size = 'large' #enter 'small', 'medium', 'large' or 'maximum'
    #beacon_setup.beacon_location returns a list with length = num_beacons giving the x,y location and depth buried
    beacon_location_vector = beacon_setup.beacon_location(y_max,x_max,num_beacons,avalanche_size,default_beacon_range)
    print beacon_location_vector
    meh = 1
    #distance map has the following [the state of the map, distance to the beacon, if the drone can see the beacon or not]
    beacon_map = beacon_setup.distance_calculator(beacon_location_vector,state_space)
    meh = 1
    beacon_setup.plot_beacon_grid(beacon_location_vector,beacon_map,x_max,y_max,z_max)
