# This file contains all the required routines to make an A* search algorithm.
#
__author__ = '1708069'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2023 - 2024
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    expanded_paths = []
    for connection in map.connections[path.last]:
        new_route = path.route.copy()
        new_route.append(connection)
        new_path = Path(new_route)
        new_path.g = path.g
        new_path.h = path.h
        new_path.f = path.f
        expanded_paths.append(new_path)
    return expanded_paths


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    valid_paths=path_list.copy()

    for path in path_list:
        for element in path.route:
            temp_route = path.route.copy()
            temp_route.remove(element)
            if element in temp_route:
                valid_paths.remove(path)
                break

    return valid_paths


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    return expand_paths + list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    paths_to_visit = [Path([origin_id])]
    
    while paths_to_visit and paths_to_visit[0].last != destination_id:
        current_path = paths_to_visit.pop(0)
        expanded = expand(current_path, map)
        expanded = remove_cycles(expanded)
        paths_to_visit = insert_depth_first_search(expanded, paths_to_visit)
    
    if paths_to_visit and paths_to_visit[0].last == destination_id:
        return paths_to_visit[0]
    return "No solution exists"


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    return list_of_path + expand_paths


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    paths_to_visit = [Path([origin_id])]
    
    while paths_to_visit and paths_to_visit[0].last != destination_id:
        current_path = paths_to_visit.pop(0)
        expanded = expand(current_path, map)
        expanded = remove_cycles(expanded)
        paths_to_visit = insert_breadth_first_search(expanded, paths_to_visit)
    
    if paths_to_visit and paths_to_visit[0].last == destination_id:
        return paths_to_visit[0]
    return "No solution exists"


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    for path_i in expand_paths:
        cost = 0
        match type_preference:
            case 0: # Adjacency
                cost = 1
            case 1: # minimum Time
                cost = (map.connections[path_i.penultimate][path_i.last])
            case 2: # minimum Distance
                if map.stations[path_i.penultimate]['name'] != map.stations[path_i.last]['name']:
                    time = map.connections[path_i.penultimate][path_i.last]
                    vel = map.velocity[map.stations[path_i.penultimate]["line"]]
                    cost = time * vel
            case 3: # minimum Transfers
                if map.stations[path_i.last]["line"] == map.stations[path_i.route[-2]]["line"]:
                    cost = 0
                else:
                    cost =1
        path_i.update_g(cost)
    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    for path_i in expand_paths:
        inserted = False
        
        for index, path in enumerate(list_of_path):
            if path_i.g < path.g:
                list_of_path.insert(index, path_i)
                inserted = True
                break
        if not inserted:
            list_of_path.append(path_i)
    
    return list_of_path


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path([origin_id])]

    while (list_of_path and (list_of_path[0].last != destination_id)):
        first = Path(list_of_path[0].route.copy())
        first.g = list_of_path[0].g
        list_of_path.remove(first)
        expanded = expand(first, map)
        expanded = remove_cycles(expanded)
        if (expanded):
            expanded = calculate_cost(expanded, map, type_preference)
            list_of_path = insert_cost(expanded, list_of_path)

    if (list_of_path[0].last == destination_id):
        return list_of_path[0]
    return ("No solution exist")


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """

    for path_i in expand_paths:
        match type_preference:
            case 0: # Adjacency
                if path_i.route[-1] == destination_id: heu = 0
                else: heu = 1
            case 1: # minimum Time
                dis = math.sqrt((map.stations[destination_id]['x'] - map.stations[path_i.route[-1]]['x']) ** 2 + (map.stations[destination_id]['y'] - map.stations[path_i.route[-1]]['y']) ** 2)
                mx_vel = max(map.velocity.values())
                heu = dis / mx_vel
            case 2: # minimum Distance
                heu = math.sqrt((map.stations[destination_id]['x'] - map.stations[path_i.route[-1]]['x']) ** 2 + (map.stations[destination_id]['y'] - map.stations[path_i.route[-1]]['y']) ** 2)
            case 3: # minimum Transfers
                if map.stations[path_i.last]["line"] == map.stations[path_i.route[-2]]["line"]:
                    heu = 0
                else:
                    heu =1
        path_i.update_h(heu)
    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path_i in expand_paths:
        path_i.update_f()
    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """
    new_paths = expand_paths.copy()
    list_of_paths = list_of_path.copy()
    for idx in range(len(expand_paths)):
        if expand_paths[idx].last in visited_stations_cost:
            if visited_stations_cost[expand_paths[idx].last] > expand_paths[idx].g:
                for path_alrdy in list_of_path:
                    if path_alrdy.last == expand_paths[idx].last: list_of_paths.remove(path_alrdy)
                visited_stations_cost[expand_paths[idx].last] = expand_paths[idx].g
            else: new_paths.remove(expand_paths[idx])
        else: visited_stations_cost[expand_paths[idx].last] = expand_paths[idx].g
    return new_paths, list_of_paths, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    for path_i in expand_paths:
        inserted = False
        for index, path in enumerate(list_of_path): 
            if path_i.f < path.f:
                list_of_path.insert(index, path_i)
                inserted = True
                break
        if not inserted:
            list_of_path.append(path_i)
    return list_of_path


def distance_to_stations(coord, map):
    """
        From coordinates, it computes the distance to all stations in map.
        Format of the parameter is:
        Args:
            coord (list): Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            (dict): Dictionary containing as keys, all the Indexes of all the stations in the map, and as values, the
            distance between each station and the coord point
    """

    distances = {}

    for station in map.stations:
        distances[station] = euclidean_dist(coord, (map.stations[station]['x'], map.stations[station]['y']))

    return distances



def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path([origin_id])]
    visited_stations_cost = {origin_id: 0}
    while (list_of_path) and (list_of_path[0].last != destination_id):
        current_path  = list_of_path[0]
        list_of_path.remove(current_path )
        expanded  = expand(current_path , map)
        expanded  = calculate_cost(expanded , map, type_preference)
        expanded  = remove_cycles(expanded )
        expanded  = calculate_heuristics(expanded , map, destination_id, type_preference=type_preference)
        expanded  = update_f(expanded )
        expanded , list_of_path, visited_stations_cost = remove_redundant_paths(expanded , list_of_path, visited_stations_cost)
        list_of_path = insert_cost_f(expanded , list_of_path)
    if (list_of_path): return list_of_path[0]
    else: return "no solution"


def Astar_improved(origin_coord, destination_coord, map):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_coord (list): Two REAL values, which refer to the coordinates of the starting position
            destination_coord (list): Two REAL values, which refer to the coordinates of the final position
            map (object of Map class): All the map information

        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_coord to destination_coord
    """
    # Add new stations:
    new_line = len(map.velocity) + 1

    start_id = 0
    destiny_id = -1

    map.add_station(start_id, "START", new_line, origin_coord[0], origin_coord[1])
    map.add_station(destiny_id, "DESTINY", new_line, destination_coord[0], destination_coord[1])

    map.velocity[new_line] = 5 # Walking speed

    map.connections[start_id] = {}
    for i in range(0, len(map.stations)-1):
        map.connections[start_id][i] = euclidean_dist(origin_coord, (map.stations[i]['x'], map.stations[i]['y']))/5
        map.connections[i][destiny_id] = euclidean_dist(destination_coord, (map.stations[i]['x'], map.stations[i]['y']))/5

    god = Astar(start_id, destiny_id, map, 1)
    god.route = [0] + god.route[1:-1] + [-1]
    return god


################################################################################################

if __name__ == "__main__":

    if (0):
        print("Proving:")
        lista=[1,2,3]
        print(f"lista\n{lista}\n\n")
        print(f"1 in lista: {1 in lista}\n0 in lista: {0 in lista}\n\n")
        lista.remove(1)
        print(f"lista\n{lista}\n\n")
        print(f"1 in lista: {1 in lista}\n0 in lista: {0 in lista}")
        print("\n\n")

    # Some adjustments:
    filename = '../CityInformation/Lyon_bigCity/Stations.txt'
    time_filename = '../CityInformation/Lyon_bigCity/Time.txt'
    vel_filename = '../CityInformation/Lyon_bigCity/InfoVelocity.txt'

    el_mapa = read_station_information(filename)
    el_mapa.connections = read_cost_table(time_filename)
    el_mapa.add_velocity(read_information(vel_filename))
    
    # Initialize star and destination coordinates as lists for mutability
    star_coord = [0, 0]
    des_coord = [0, 0]
    
    # Get inputs from user and convert to integers
    star_coord[0] = int(input("Start x: "))
    star_coord[1] = int(input("Start y: "))
    des_coord[0] = int(input("Destiny x: "))
    des_coord[1] = int(input("Destiny y: "))

    # Call the A* algorithm with the user-defined coordinates
    route = Astar_improved(tuple(star_coord), tuple(des_coord), el_mapa)
    
    print(f"{route.route=}")