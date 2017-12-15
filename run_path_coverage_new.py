import graph_search_improved


def run_path_coverage(map_path, cliff_height):
    g = graph_search_improved.GridMap(map_path, cliff_height)

    res = graph_search_improved.path_coverage(g, g.init_pos, g.transition, graph_search_improved._ACTIONS)
    # print res[0]
    g.display_map_new(res)
        # print(res[0], res[1])
    # else:
    #     g.display_map_new(res[0][1], res[1])
    #     print(res[0][1], res[1])

    g.display_cell_values()
    g.display_cell_height()


if __name__ == '__main__':
    run_path_coverage('./map1.txt', 1.0)

