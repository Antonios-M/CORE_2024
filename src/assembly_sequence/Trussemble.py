## Dependecies used for type hinting (not available by default in Python 3.9.10)
from __future__ import annotations
from typing import List, Tuple

## McNeel-related dependencies
import rhinoscriptsyntax as rs 
import Grasshopper as gh
import Grasshopper.Kernel as ghk
from Grasshopper.Kernel.Data import GH_Path
import Rhino.Geometry as rg

## General utilities
import networkx as nx
from collections import Counter
import math
import numpy as np
import itertools
from itertools import combinations

class Trussemble:
    """
    A class to produce an assembly sequence of an input structural latice of beams (lines). The assembly sequence is made 
    such that it can be built with two robots (co-bots) and employs clever strategies to shorten the assembly path, but 
    also maintain structural rigidity along every step of the way. 

    Requirements: Every node has to connect to at least three other nodes, unless it is a 'rigid' node, (a.k.a a support node)

    Inputs ---------------------------------------------------------------------------------------------------------------->

    in_lines (required): the set of beams that are the lattice frame, given as Rhino.Geometry.Curve or Rhino.Geometry.Line

    in_rigid_points (required): the set of points that define the support points of the structure, script will still work 
        if the points are not part of the endpoints of the beams (closest endpoint will be assigned as rigid point),
        this is done to avoid tolerence issues between the two inputs.



    in_start_point (not required): the start point from which the script will find the starting rigid points. This point
        does not need to be part of the rigid points or any nodes in the lattice, it can be anywhere in space. The script
        will chose the closest three rigid points which share a neighbour and will start assembling from there.


    Returns -------------------------------------------------------------------------------------------------------------->

    assembly_steps: a gh.DataTree of type rhino.Line, with every branch being a list of lines, representing one step of the assembly sequence

    processed_nodes: a list of rg.Point3d of equal length to assembly_steps. Each point in the list corresponds to each branch in assembly_steps
        and is also the connecting node for all lines in that branch. 

    lines_indices: a gh.DataTree of type int which has is identical to assembly_steps, but instead of each item being a curve, each item is the 
        index of the curve in the input in_lines list. This is useful to reference each assembly_step to the input lines



    """
    def __init__(self, lines: List(rg.Curve), rigid_points: List(rg.Point3d), start_point: rg.Point3d) -> None:
        ##validate inputs
        if not all(isinstance(line, rg.Line) for line in in_lines) or not in_lines:
            raise ValueError('Input Lines are not Lines')
        if not all(isinstance(point, rg.Point3d) for point in in_rigid_points) or not in_rigid_points:
            raise ValueError('Input Rigid Points are not Points')
        if not isinstance(in_start_point, rg.Point3d) or not in_start_point:
            raise ValueError('Input start point is not a point')

        ## inputs
        self.lines = lines
        self.points, self.edges_list, self.graph = self.__get_line_connectivity()
        self.rigid_points = rigid_points
        self.rigid_indices = self.__get_rigid_point_indices()
        self.start_point = start_point
        ## outputs
        self.assembly_steps = gh.DataTree[object]()
        self.assembly_step_line_indices = gh.DataTree[object]()
        self.processed_nodes = []
    
    def __average_point(self, pt1: rg.Point3d, pt2: rg.Point3d, pt3: rg.Point3d) -> rg.Point3d:
        """
        Returns the average point as a rg.Point3d, given three input points
        """ 
        # Calculate the average of the x, y, and z coordinates
        avg_x = (pt1.X + pt2.X + pt3.X) / 3
        avg_y = (pt1.Y + pt2.Y + pt3.Y) / 3
        avg_z = (pt1.Z + pt2.Z + pt3.Z) / 3
        
        # Return the average point as a Point3d
        average_point = rg.Point3d(avg_x, avg_y, avg_z)
        return average_point

    def __distance(self, p1: rg.Point3d, p2: rg.Point3d) -> float:
        """
        returns distance between two points, calculated using numpy which prevents errors due to tolerance or too large numbers
        """
        np_p1 = np.array((p1.X, p1.Y, p1.Z))
        np_p2 = np.array((p2.X, p2.Y, p2.Z))

        distance = np.linalg.norm(np_p1 - np_p2)

        return distance

    def __get_rigid_point_indices(self) -> List(int):
        """
        Returns the indices of the input rigid points as they map onto self.points, which are indexed in self.graph
        """
        rigid_indices = [] ## init rigid point indices list
        points_array = np.array([[round(pt.X,3), round(pt.Y,3), round(pt.Z,3)] for pt in self.points]) ## make rigid points into np array, allows easier and more reliable distance calculation regardless of document units

        for rigid_point in self.rigid_points:
            if rigid_point in self.points: ## if rigid point is in self.points (i.e if distance between the corresponding point is 0)
                rigid_indices.append(self.points.index(rigid_point))

            else: ## if distance between them is not 0
                rigid_point_array = np.array([rigid_point.X, rigid_point.Y, rigid_point.Z]) ## rigid point as np array
                distances = np.linalg.norm(points_array - rigid_point_array, axis=1) ## distance between rigid point and all self.points
                rigid_idx = np.argmin(distances) ## min distance between rigid point and one point in self.points
                rigid_indices.append(rigid_idx) ## add index of closest rigid point to rigid_indices
        return rigid_indices

    def __get_line_connectivity(self) -> Tuple( List(rg.Point3d), List(Tuple(int,int), nx.Graph())):
        """
        Create a list of nodes, a list of connections between nodes and a graph of those connections

        this is a private method and uses self.lines as input

        Returns a Tuple of a list of rg.Point3d (the nodes), a list of tuples (the graph edges) and a netwroxk graph
        """
        lines = self.lines
        edges_list = [] # use a list to store the edges (G.edges() is not used as its edges are sorted and so the indices do not map directly to input in_lines)
        unique_points = {}  # use a dictionary to map points to their indices


        for line_index, line in enumerate(lines):
            # get the start and end points of the line
            start_point = line.From
            end_point = line.To
            
            # CRUCIAL: convert to rg.Point3d and ROUND COORDINATES (prevents issue of not being able to find matching point because of document tolerance)
            start_point3d = rg.Point3d(round(start_point.X, 3), round(start_point.Y, 3), round(start_point.Z, 3))
            end_point3d = rg.Point3d(round(end_point.X, 3), round(end_point.Y, 3), round(end_point.Z, 3))


            if start_point3d not in unique_points: # add points to the unique points dictionary if they don't already exist
                unique_points[start_point3d] = len(unique_points) ## assign point to its index
            if end_point3d not in unique_points:
                unique_points[end_point3d] = len(unique_points)
            
            # get the indices of the start and end points
            start_index = unique_points[start_point3d]
            end_index = unique_points[end_point3d]

            # append the indices to the edges list
            edges_list.append((start_index, end_index))

        # convert unique points to a list of Point3d
        unique_points_list = list(unique_points.keys()) ## get point keys from dictionary
        G = nx.Graph() ## init networkx graph
        G.add_edges_from(edges_list) ## add edges to graph

        return unique_points_list, edges_list, G

    def __can_node_be_rigid(self, node: int) -> bool:
        """
        This is used to test if a candidate meets the minimum requirements to be considered rigid

        if these minimum requirements are met, the candidate is tested for hinge-behaviour
        """
        if node in self.rigid_indices: ## node can be rigid if it is already rigid
            return True
        else: ## node can be rigid if it is not already rigid and has AT LEAST 3 neighbours in processed_nodes
            graph = self.graph
            neighbours = list(graph.neighbors(node)) ## neighbours of node
            count_existing_neighbours = sum(1 for neighbour in neighbours if neighbour in self.processed_nodes) ## list of 1s, length of list = neighbours of node that are already processed
            return count_existing_neighbours >= 3

    def __get_first_move(self) -> Tuple( List(int), int ):
        """
        Function to find first move given a start point
        """ 

        ## define inputs
        start_point = self.start_point
        rigid_ids = self.rigid_indices
        points = self.points
        graph = self.graph

        ## intialise utility variables and outputs
        min_avg_distance_to_start = float('inf') ## min avg distance of 3 rigid points to start point
        neighbours_to_connect_to = [] ## neighbours that the 3 rigid points chosen can all connect to (shared neighbours)
        first_supports = None ## first three rigid points, supports, that are connected by the neighbours_to_connect_to

        def find_common_neighbour_combinations(graph: nx.Graph(), rigid_points_indices: List(int)) -> Tuple( List(set(int,int,int)), List(int)):
            """
            Finds combinations of 3 rigid points which share at least one neighbour
            """
            valid_combinations = [] ## initialse list to store sets of 3 neighbours
            common_neighbors = [] ## common neighbour of each set of neighbours for the sets in valid_combinations
            
            # generate all combinations of three nodes from rigid_points
            for comb in combinations(rigid_points_indices, 3):
                # get neighbors for each node in the combination
                neighbors_1 = set(graph.neighbors(comb[0]))
                neighbors_2 = set(graph.neighbors(comb[1]))
                neighbors_3 = set(graph.neighbors(comb[2]))
                
                # find common neighbors
                common_neighbor = neighbors_1 & neighbors_2 & neighbors_3
                
                # if there is at least one common neighbor, add the combination to the result
                if common_neighbor:
                    valid_combinations.append(comb)
                    common_neighbors.append(list(common_neighbor)[0])

            return valid_combinations, common_neighbors
                
        ## get all candidates of sets of 3 rigid point nodes where all 3 share at least one neighbour
        starting_three_candidates, common_neighbours = find_common_neighbour_combinations(graph, rigid_ids)

        ## iterate through all sets of three rigid point nodes that share at least one neighbour
        for i, starting_three in enumerate(starting_three_candidates):

            ## get 3 sepparate points from each set of rigid point nodes
            p1 = points[starting_three[0]]
            p2 = points[starting_three[1]]
            p3 = points[starting_three[2]]

            centre = self.__average_point(p1, p2, p3) ## get average location of the three points

            avg_distance = self.__distance(start_point, centre) ## compute distance between average location of set of rigid points to start point

            ## if the average distance of the set of neighbours is less than the one before it, update the distance and store the combination of the first three supports and also the neighbour_to_connect_to
            if avg_distance < min_avg_distance_to_start:
                min_avg_distance_to_start = avg_distance
                first_supports = starting_three
                neighbours_to_connect_to.append(list([common_neighbours[i]]))

        ## return last added neighbourt_to_connect_to, [-1][0] is used because it is the last item added that should be considered, but the first neighbour (assuming some rigid points might share more than one neighbour) 
        neighbour_to_connect_to = neighbours_to_connect_to[-1][0]

        self.processed_nodes.extend(first_supports) ## add first three supports to processed nodes
        self.processed_nodes.append(neighbour_to_connect_to) ## add shared neighbour to processed nodes
        return list(first_supports), neighbour_to_connect_to ## returns the first three supports and their common neighbour

    def __check_edges(self, sub_graph_nodes: List(np.int64)):
        """
        Function that returns all edges in a graph that exist between sub_graph_nodes
        """
        graph = self.graph
        edges_to_add = [] ## init list of edges

        ## add edge to list if edge exists in graph (works only for un-directed graph, as the roof lattice is modelled as an un-directed graph)
        for node1, node2 in combinations(sub_graph_nodes, 2):
            if graph.has_edge(node1, node2):
                edges_to_add.append((node1, node2))

        return edges_to_add

    def __find_unprocessed_neighbors_subgraphs(self) -> List(int):
        """
        A function that returns the connected component in un-processed nodes which includes the node that is closest to the last added node

        Explanation points:

            1) The processed nodes form a subgraph of un-processed nodes, i.e all the nodes still remaining to be assembled
            2) The nodes in the un-processed nodes which are 'candidates', i.e have more than 3 connections to processed nodes are considered as the subgraph
            3) the subgraph can be conceptualised as the 'front' of the processed nodes, i.e the nodes which are not yet processed but are 1 connection away from being processed
            4) this 'front' can be broken down into connected components
            5) a connected component is a set of nodes which form a subgraph where no node is disconnected from that subgraph
            6) The front can then be thought of as sepparate subgraphs, if those subgraphs do not share connections with eachother
            7) effectively this reduces the number of candidates in each step to the ones that are connected with eachother and limits the 'jumping' of the robot from one area of the lattice to another

        """

        ## get inputs needed for the subgraphs to be found
        graph = self.graph
        processed_nodes = self.processed_nodes
        processed_nodes_set = set(processed_nodes)
        points = self.points
        
        # step 1a: find neighbours of processed nodes that arent yet processed (i.e the 'front')
        unprocessed_neighbors = set()

        # step 1b: get the neigbours of the last processed nodes
        last_node = processed_nodes[-1]
        neighbors_last_node = set(graph.neighbors(last_node))
        unprocessed_neighbors_last_node = neighbors_last_node - processed_nodes_set

        # step 1c: find unprocessed neighbors of all other processed nodes
        for node in processed_nodes_set:
            if node == last_node:
                continue  # Skip the last node as we already processed it
            neighbors = set(graph.neighbors(node))
            unprocessed_neighbors.update(neighbors - processed_nodes_set)

        # step 1d: add unprocessed neighbors of the last node to the main unprocessed set
        unprocessed_neighbors.update(unprocessed_neighbors_last_node)

        # step 2a: initialise list of subgraphs
        subgraphs = []
        
        # step 2b: get subgraph that includes only unprocessed neighbors
        unprocessed_subgraph = graph.subgraph(unprocessed_neighbors)
        
        # step 2c: find connected components in the unprocessed subgraph
        for component in nx.connected_components(unprocessed_subgraph):
            subgraphs.append(list(component))
        
        # step 3: initialise sorted subgraphs, to sort subgraphs based on their connection to last added node
        subgraphs_sorted = []
        
        # step 3a: subgraph connected to the last processed node
        subgraph_last_node = [comp for comp in subgraphs if any(node in unprocessed_neighbors_last_node for node in comp)]
        
        # step 3b: remaining subgraphs (not connected to the last node)
        other_subgraphs = [comp for comp in subgraphs if comp not in subgraph_last_node]
        
        # step 4: sort subgraphs so that the first subgraph is connected to the last added node
        if subgraph_last_node:
            subgraphs_sorted.extend(subgraph_last_node)
            subgraphs_sorted.extend(other_subgraphs)
            return subgraphs_sorted[0]

        else: ## if the last added node is not connected to any of the subgraphs (occurs at corners when last added node is corner node)
            closest_distance = float('inf')
            first_comp = None
            for comp in subgraphs:
                for node in comp:
                    distance = self.__distance(points[last_node], points[node]) ## pick subgraph which incldues node with shortest distance from last added node.
                    if distance < closest_distance:
                        first_comp = comp
                        closest_distance = distance
            return first_comp

    def __get_possible_connections(self, node: int) -> List(int):
        """
        returns neighbours of node that are not in processed nodes
        """
        graph = self.graph
        can_connect_to = [neighbour for neighbour in graph.neighbors(node) if neighbour in self.processed_nodes]
        return can_connect_to

    def __sort_rigid_candidates(self, candidates: List[int]) -> List[int]:
        """
        Sorts candidates that can be rigid by their distance to the last added point.
        """
        origin = self.processed_nodes[-1]  # Get the last connected node
        points = self.points

        # Sort candidates by their distance to the origin point, using a lambda function
        sorted_candidates = sorted(candidates, key=lambda c: self.__distance(points[origin], points[c]))
        return sorted_candidates

    def __get_sorted_supports(self, supports: List(rg.Point3d)):
        """
        Sorts the supports of a candidate node by considering the distance of all supports from eachother and their triangle area. The points should have a small area but a large distance from eachother
        """
        support_combinations = list(itertools.combinations([x for x in range(len(supports))], 3)) ## combinations of 3 elements in the supports, given as indices
        score_per_combination = {} ## each combination has a score by how rigid it is
        result_supports = [] ## init list to store the order of supports that is optimal

        def get_combined_distance_of_supports(supports: List(rg.Point3d)) -> float:
            """
            Gets a combined distance metric between all points in the supports list, i.e a high combined distance means the points are far away from each other
            Calculation approach might seem long-winded but rs.Distance() couldnt be used because of constraints with too large/small numbers and doing this with matrices is less verbose than calculating multiple distances consequently and averaging
            """
            support_coords = np.array([[pt.X, pt.Y, pt.Z] for pt in supports])
            num_supports = len(support_coords)
            diff = support_coords[:, np.newaxis, :] - support_coords[np.newaxis, :, :] ## calculates the difference ( as a vector (x,y,z)) in each X, Y and Z for each pair of points (results in 3d matrix) i.e pairwise difference of points
            distances = np.linalg.norm(diff, axis=2) ## get euclidian distance between all pairs of points 
            distances += np.eye(num_supports) * 1e-8 ## identity matrix, I,  gives the distance between any two points, multiply by small number to prevent 0s, as I(x,x) = 0, i.e distance of any point to itself is 0
            combined_distance = np.prod(distances) ## get product of all pairwise distances between any two points
            return combined_distance

        def triangle_area(rs_pointA: rg.Point3d, rs_pointB: rg.Point3d, rs_pointC: rg.Point3d) -> float:
            """
            Returns area of triangle formed by two points
            """
            A = np.array([rs_pointA.X, rs_pointA.Y, rs_pointA.Z])
            B = np.array([rs_pointB.X, rs_pointB.Y, rs_pointB.Z])
            C = np.array([rs_pointC.X, rs_pointC.Y, rs_pointC.Z])
            area = 0.5 * np.linalg.norm(np.cross(B - A, C - A)) ## triangle area = 0.5 x (b x h)
            return area

        ## iterate through all support combinations and add score to each combinations
        for c in support_combinations:
            supports_list = [supports[c[0]], supports[c[1]], supports[c[2]]]

            combined_distance = float(get_combined_distance_of_supports(supports_list))
            area = float(triangle_area(supports_list[0], supports_list[1], supports_list[2]))
            if area == 0.0:
                score = combined_distance ## score is higher if combined distance is higher
            else:
                score = combined_distance / area ## score is lower if area is higher
            score_per_combination[c] = score
        
        score_per_combination = dict(sorted(score_per_combination.items(), key=lambda item: item[1])) ## sort support combinations by score

        support_combinations = list(score_per_combination.keys())

        ## add sorted supports to result support
        for support_combination in support_combinations:
            result_support = list(support_combination)
            for i, support in enumerate(supports):

                if i not in support_combination:
                    result_support.append(i)

                result_supports.append(result_support)

        return result_supports[0]

    def __check_if_door(self, support_nodes: List(int), load_node: int) -> bool or List(int):
        """
        Check if a composition of supports nodes and a load node produce hinge-behavour
        """
        ## get inputs
        support_points = [self.points[x] for x in support_nodes]
        load_point = self.points[load_node]
        combination = []

        def point_plane_distance(plane_pts: List(rg.Point3d), test_pt: rg.Point3d) -> float:
            """
            gets distance of a point to a plane (couldnt be done with rhinoscriptsyntax because of crashing with too large or small numbers)
            """
            A, B, C = plane_pts ## input plane pts are always 3 because 2 supports and 1 load point used to make plane
            ## get plane vectors
            AB = B - A
            AC = C - A
            normal_vector = rg.Vector3d.CrossProduct(AB, AC) ## get normal of vector by using cross product of two co-planar vectors
            if normal_vector.IsZero:
                return 0.0
            normal_vector.Unitize()
            D = - (normal_vector.X * A.X + normal_vector.Y * A.Y + normal_vector.Z * A.Z)
            numerator = abs(normal_vector.X * test_pt.X + normal_vector.Y * test_pt.Y + normal_vector.Z * test_pt.Z + D) ## eq. to find distance from one point to a plane (assuming normal vec is unitised)
            distance = numerator  # since normal_vector is unitised, denominator is 1
            return distance

        def distance_to_centroid(pts: List(rg.Point3d), test_pt: rg.Point3d) -> float:
            """
            Distance calc wrapped in function to make in-line code more readable
            """
            centroid = rg.Point3d((pts[0].X + pts[1].X + pts[2].X) / 3,
                                  (pts[0].Y + pts[1].Y + pts[2].Y) / 3,
                                  (pts[0].Z + pts[1].Z + pts[2].Z) / 3)
            distance = self.__distance(test_pt, centroid)
            return distance

        def is_door(three_supports: List(rg.Point3d), load_point: rg.Point3d, span_height_ratio=0.25) -> bool:
            distance = point_plane_distance(three_supports, load_point) ## get distance to plane formed by two supports and load point
            effective_span = distance_to_centroid(three_supports, load_point) ## effective span is average distance of each support to the load point
            if effective_span == 0:
                return False
            ratio = distance / effective_span ## if distance between third point and the plane is smaller than 25% of the span then there can be hinge-like behaviour (which is further validated by Karamba FEA)
            return ratio <= span_height_ratio

        first_three = support_points[:3]

        ## if first three supports do not behave like a hinge return the supports
        if not is_door(first_three, load_point):
            return support_nodes
        else: ## if first three supports do behave like a hinge, try to find another combination that doesnt behave like a hinge
            for comb in combinations(support_nodes, 3):
                comb_points = [self.points[x] for x in list(comb)]

                if not is_door(list(comb_points), load_point):
                    combination.extend(list(comb))
                    missing = [x for x in support_nodes if x not in list(comb)]
                    if missing:
                        combination.extend(missing)
                    return combination
            ## if not combination is found, then return True (True = behaves like a hinge, and can't be fixed)
            return True

    def __find_edge_index(self, edge: Tuple(int,int)) -> int:
        """
        Gets index of an edge in a graph, used to create lines_indices output
        """
        # get a list of all edges in the graph
        edges_list = self.edges_list
        
        # check if the edge exists in the edge list
        if edge in edges_list or (edge[1], edge[0]) in edges_list:
            return edges_list.index(edge) if edge in edges_list else edges_list.index((edge[1], edge[0]))
        
        return -1  # edge doesn't exist

    def assemble(self) -> gh.DataTree(List(rg.Curve)):
        """
        Main funcion of class that creates assembly step sequence
        """

        ## get inputs 
        graph = self.graph
        points = self.points
        assembly_steps = self.assembly_steps
        assembly_lines_indices = self.assembly_step_line_indices

        ## start while loop that runs until all edges are processed
        while True:
            ## assembly process first step
            if len(self.processed_nodes) == 0:
                first_supports, first_neighbour = self.__get_first_move() ## get first move
                ## initialise lines to add to move and line indices to add
                lines_to_add = []
                lines_indices = []
                
                ## create line for each edge
                for support in first_supports:
                    p1 = points[support]
                    p2 = points[first_neighbour]

                    line = rs.AddLine(p1, p2)
                    lines_to_add.append(line)
                    lines_indices.append(self.__find_edge_index((int(support), int(first_neighbour))))
                
                ## check if missing edges exist (edges between supporting nodes)
                missing_edges = self.__check_edges(first_supports)
                for missing_edge in missing_edges:
                    p1 = points[missing_edge[0]]
                    p2 = points[missing_edge[1]]

                    line = rs.AddLine(p1,p2)
                    lines_to_add.append(line)
                    lines_indices.append(self.__find_edge_index((int(missing_edge[0]), int(missing_edge[1]))))

                ## add lines to assembly steps
                path = GH_Path(0)
                assembly_steps.AddRange(lines_to_add, path)
                assembly_lines_indices.AddRange(lines_indices, path)

            ## assembly process middle steps
            elif len(self.processed_nodes) < len(points):
                non_rigid_candidates = self.__find_unprocessed_neighbors_subgraphs() ## get connected component of unprocessed nodes
                rigid_candidates = [candidate for candidate in non_rigid_candidates if self.__can_node_be_rigid(candidate)] ## check if nodes in connected component can be rigid
                sorted_rigid_candidates = self.__sort_rigid_candidates(rigid_candidates) ## sort rigid candidates based on their distance to last added node


                for candidate in sorted_rigid_candidates:
                    lines_to_add = []
                    lines_indices = []
                    nodes_to_connect_to = self.__get_possible_connections(candidate) ## connections to processed nodes

                    ## edge case: next point to connect to is rigid and it only has one connection (i.e last added node is rigid)
                    ## this makes the simulation only follow the rigid points and doesnt complete and steps of the spanning nodes
                    if len(nodes_to_connect_to) == 1 and nodes_to_connect_to[0] in self.rigid_indices:
                        continue

                    ## if candidate is rigid point (no need for hinge-behaviour)
                    if candidate in self.rigid_indices:
                        for node in nodes_to_connect_to:
                            line = rs.AddLine(points[candidate], points[node])
                            lines_indices.append(self.__find_edge_index((int(candidate), int(node))))
                            lines_to_add.append(line)

                        ## add node to processed nodes and add step to assembly steps
                        self.processed_nodes.append(candidate)
                        path = GH_Path(assembly_steps.BranchCount)
                        assembly_steps.AddRange(lines_to_add, path)
                        assembly_lines_indices.AddRange(lines_indices, path)

                            
                    ## if candidate is not a rigid point
                    else:
                        nodes_sorted_indices = self.__get_sorted_supports([points[x] for x in nodes_to_connect_to]) ## sort supports based on combined distance and triangle area, returns sorting order 
                        sorted_support_nodes = [nodes_to_connect_to[x] for x in nodes_sorted_indices] ## get nodes of supports based on sorting order

                        ## check if there is hinge-behaviour, if there is and it cant be resolved, stop considering candidate and move to next candidate
                        if self.__check_if_door(sorted_support_nodes, candidate) == True:
                            continue

                        ## if there is hinge-behaviour but can be solved, return the reordered supports so that there isnt hinge-behaviour
                        else: 
                            sorted_support_indices = self.__check_if_door(sorted_support_nodes, candidate)

                        ## make lines from supports and candidate
                        for point_index in sorted_support_indices:
                            line = rs.AddLine(points[candidate], points[point_index])
                            lines_indices.append(self.__find_edge_index((int(candidate), int(point_index))))
                            lines_to_add.append(line)
                            
                        ## add candidate to processed nodes and step to assembly steps
                        self.processed_nodes.append(candidate)
                        path = GH_Path(assembly_steps.BranchCount)
                        assembly_steps.AddRange(lines_to_add, path)
                        assembly_lines_indices.AddRange(lines_indices, path)
            else:
                return assembly_steps


## init Trussemble class
my_truss = Trussemble(in_lines, in_rigid_points, in_start_point)

## specify outputs
assembly_steps = my_truss.assemble()
processed_nodes = my_truss.processed_nodes
line_indices = my_truss.assembly_step_line_indices
