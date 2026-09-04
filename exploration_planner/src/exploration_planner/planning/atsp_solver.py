"""
ATSP Solver using OR-Tools for global exploration planning
"""

import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class ATSPSolver:
    """
    Asymmetric TSP solver for region sequence optimization
    """
    
    def __init__(self, time_limit_sec=2):
        self.time_limit_sec = time_limit_sec
    
    def solve(self, cost_matrix):
        """
        Solve ATSP problem
        
        Args:
            cost_matrix: N×N numpy array of costs
        
        Returns:
            tour: List of node indices in visit order (excluding start)
        """
        N = len(cost_matrix)
        
        if N <= 1:
            return list(range(N))
        
        # Create routing model
        manager = pywrapcp.RoutingIndexManager(N, 1, 0)
        routing = pywrapcp.RoutingModel(manager)
        
        # Define cost callback
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(cost_matrix[from_node][to_node] * 100)
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        # Search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = self.time_limit_sec
        
        # Solve
        solution = routing.SolveWithParameters(search_parameters)
        
        if not solution:
            return None
        
        # Extract tour
        tour = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            tour.append(node)
            index = solution.Value(routing.NextVar(index))
        
        return tour[1:]  # Exclude start node
