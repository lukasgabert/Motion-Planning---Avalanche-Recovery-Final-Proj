import avalanche_map as a_map
# import numpy as np
# import temp


if __name__ == '__main__':
    # simple_map = a_map.AvalancheMap('Maps\\snow2.stl', 0.5)
    # simple_map.save_map()

    simple_map = a_map.AvalancheMap(voxel_delta=0.5)
    simple_map.load_map('Maps\\default_map.txt')

