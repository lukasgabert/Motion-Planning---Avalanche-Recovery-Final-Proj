import avalanche_map as a_map
import copy

if __name__ == '__main__':

    #setting up grid maps and beacon maps
    simple_map = a_map.AvalancheMap(voxel_delta=0.5)
    simple_map.load_map('Maps\\default_map.txt')
    master_visited = []
    num_beacons = 2
    default_beacon_range = 180
    avalanche_size = 'small' #enter 'small', 'medium' or 'large'
    #simple_map.beacon_location returns a list with length = num_beacons giving the x,y and z locations
    beacon_locations = simple_map.beacon_location(num_beacons,avalanche_size,default_beacon_range)
    print 'beacon locations: ', beacon_locations

    #simple_map.distance_calculator calculates the distances to each drone, if it is within the search range, it set to
    #true, otherwise, false.
    simple_map.distance_calculator(beacon_locations)
    #simple_map.plot_beacon_grid(beacon_locations)
    # Starting Point, active beacon, distance and action for testing
    x = 0
    y = 0
    z = 12
    current_action = 'r'
    current_state = []
    active_beacon_index = 0
    current_state.append(x)
    current_state.append(y)
    current_state.append(z)
    dist_2_beacon = simple_map.beacon_map[x][y][z][active_beacon_index][1]
    #print 'dist_2_beacon: ', dist_2_beacon
    #print 'initial cur_action: ', cur_action
    #Local search for the beacon
    master_visited = []
    for beacons in beacon_locations:
        print 'active beacon: ', beacon_locations[active_beacon_index]
        #print 'active beacon index: ',active_beacon_index
        beacon_visibility = simple_map.beacon_map[x][y][z][active_beacon_index][0]
        if beacon_visibility:
                current_state,minimized_path = simple_map.beacon_search(current_state,current_action, dist_2_beacon,active_beacon_index)
    #            print minimized_path
                master_visited = master_visited + minimized_path
    #            active_beacon_index += 1
    #             if active_beacon_index < len(beacon_locations):
    #                 #print 'active beacon: ',beacon_locations[active_beacon_index]
    #                 #print 'active beacon index: ', active_beacon_index
    #                 dist_2_beacon = simple_map.beacon_map[current_state[0]][current_state[1]][current_state[2]][active_beacon_index][1]
    #                 #find the right action to take for the next beacon
    #                 options = []
    #                 #find the best action to move in from current_state
    #                 for action in simple_map.search_actions:
    #                     possible_state = simple_map.transfer_func(current_state,action)
    #                     possible_dist2next = simple_map.beacon_map[possible_state[0]][possible_state[1]][possible_state[2]][active_beacon_index][1]
    #                     options.append([possible_state, action, possible_dist2next])
    #                 #print 'current state:', current_state
    #                 #print options
    #                 d_min = 10000
    #                 for d in options:
    #                     d_choice = d[2]
    #                     if d_choice <= d_min:
    #                         current_state = d[0]
    #                         current_action = d[1]
    #                         dist_2_beacon = d[2]
    #
    #                 #print 'best: ',current_action,current_state,dist_2_beacon
    #                 master_visited = master_visited + [current_state]
    #                 meh = 2
    print master_visited
    simple_map.plot_master_path(beacon_locations,master_visited)

    meh = 1
