from flask import Flask, request, jsonify
import subprocess
from SearchAlgorithm import (
    __author__, expand, calculate_cost, calculate_heuristics, remove_cycles, depth_first_search,
    breadth_first_search, uniform_cost_search, remove_redundant_paths, distance_to_stations, Astar, Astar_improved)
from SubwayMap import Path
from utils import print_list_of_path_with_cost, read_station_information, read_cost_table, read_information
import os
import numpy as np

app = Flask(__name__)

@app.route('/run-python-script', methods=['POST'])
def run_python_script():
    # Get the coordinates from the request
    data = request.get_json()
    row = data['row']
    col = data['col']

    # Use numpy to compute the sum of the coordinates
    coordinate_sum = np.array([row, col]).sum()

    # Convert the sum back into a 1D index (row * gridSize + col)
    grid_size = 250  # This must match the gridSize in the frontend
    result_index = coordinate_sum % (grid_size * grid_size)

    # Return the result index as JSON
    return jsonify({'resultIndex': int(result_index)})

if __name__ == '__main__':
    app.run(debug=True)

