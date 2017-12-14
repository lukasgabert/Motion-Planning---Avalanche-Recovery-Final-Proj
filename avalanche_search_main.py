import beacon_setup
import math
import avalanche_map as a_map


if __name__ == '__main__':

    #setting up grid maps and beacon maps
    simple_map = a_map.AvalancheMap(voxel_delta=0.5)
    simple_map.load_map('Maps\\default_map.txt')
    num_beacons = 3
    default_beacon_range = 15
    avalanche_size = 'large' #enter 'small', 'medium' or 'large'
    #simple_map.beacon_location returns a list with length = num_beacons giving the x,y and z locations
    beacon_locations = simple_map.beacon_location(num_beacons,avalanche_size,default_beacon_range)
    print 'beacon locations: ', beacon_locations

    #simple_map.distance_calculator calculates the distances to each drone, if it is within the search range, it set to
    #true, otherwise, false.
    beacon_map = simple_map.distance_calculator(beacon_locations)
    simple_map.plot_beacon_grid(beacon_locations,beacon_map)
