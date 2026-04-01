from flask import Flask, render_template, request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

app = Flask(__name__)

# Sample predefined locations (Dropdown options)
LOCATIONS = {
    "Warehouse": (19.0760, 72.8777),
    "Andheri": (19.1136, 72.8697),
    "Bandra": (19.0596, 72.8295),
    "Dadar": (19.0176, 72.8562),
    "Powai": (19.1176, 72.9060)
}

# Distance calculation (Euclidean approx)
def compute_distance_matrix(coords):
    size = len(coords)
    matrix = []
    for i in range(size):
        row = []
        for j in range(size):
            dist = math.sqrt((coords[i][0]-coords[j][0])**2 + (coords[i][1]-coords[j][1])**2)
            row.append(int(dist * 10000))
        matrix.append(row)
    return matrix


def solve_vrp(locations, vehicle_count=2, capacity=15):
    coords = [LOCATIONS[loc] for loc in locations]
    distance_matrix = compute_distance_matrix(coords)

    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), vehicle_count, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Solve
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)

    routes = []
    if solution:
        for vehicle_id in range(vehicle_count):
            index = routing.Start(vehicle_id)
            route = []
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                route.append(locations[node])
                index = solution.Value(routing.NextVar(index))
            route.append("Warehouse")
            routes.append(route)

    return routes


@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        selected = request.form.getlist("locations")
        vehicle_count = int(request.form.get("vehicles"))

        routes = solve_vrp(["Warehouse"] + selected, vehicle_count)

        return jsonify({"routes": routes})

    return render_template("index.html", locations=list(LOCATIONS.keys())[1:])


if __name__ == "__main__":
    app.run(debug=True)
