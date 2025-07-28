import random
import math
import os

import pandas as pd
import numpy as np
import os
import networkx as nx

import networkx as nx
import csv
import math
from scipy.misc import derivative
import scipy.integrate as integrate
import numpy as np
import random
from collections import OrderedDict
import matplotlib.pyplot as plt
import scipy
import random
import collections
from itertools import islice
from math import radians, cos, sin, asin, sqrt


class Link(object):


    def __init__(self, link_id=None, length=0,
                     from_node=0, to_node=0, flow=float(0.0), free_speed=0, free_flow_travel_time=0,
                     link_id_text=str(0), route_id=0):
        self.link_id = link_id
        self.link_id_text=link_id_text
        self.length = length # ft

        self.from_node = from_node
        self.to_node = to_node
        self.timestep = 0
        self.timesteps = 60
        self.timeinterval = 15.0  # seconds
        self.free_speed = free_speed  # ft/sec = 51.33*0.681818(converting fts to mph) = 34.9977 mph, 1.4667 (converting mph to fps)
        self.free_flow_time=free_flow_travel_time
        self._time = None
        self.density = 0.0  # veh/mile (ft to miles = ft*5280.0)
        # self.k_j = 160.002  # if we set this value to decimal, we can have a very low chance that fails to derivative.
        self.k_j = 160.002/5280.0  #0316 Siwei: k_j -> veh/ft
        self.v = 0.  # mph
        self.flow = flow  # vehicles, vph(bpr)
        self.k_t = 0.0
        self.use_exact = False
        self.truncate=False
        self.link_function = "m_Greenshield"  # "bpr", "m_Greenshield","triangularFD"
        self.SO = False
        self.random_init_vol = False

        self.u_max = self.free_speed  # mph
        # self.u_min = 10.0  # mph
        self.u_min = 10.0*1.4667  #Siwei: ft/s 10mph
        
        # Edited
        # List of route_id's of this link
        self.route_id = route_id
        # Travel time penalty for changing routes
        self.transfer_penalty = 30


class Node(object):
    """
    Class for handling node object in Transportation Networks

    Parameters
    ----------
    node_id:    int
                identifier of a node

    """

    def __init__(self, node_id=0, node_XCOORD=0, node_YCOORD=0, is_stop_only=False, census_tract=0, node_in_fleetpy=0,
                 is_demand_node=0):

        self.node_id = node_id
        self.node_XCOORD = node_XCOORD
        self.node_YCOORD = node_YCOORD
        self.is_stop_only = is_stop_only
        self.census_tract = census_tract
        self.node_in_fleetpy = node_in_fleetpy
        self.is_demand_node = is_demand_node


def read_network(Network_file):
    with open(Network_file) as f:
        csvreader = csv.DictReader(f)
        nodes = {}
        links = []
        node_list = []

        for data in csvreader:
            origin_node = data["from_node"] # EDITED
            to_node = data["to_node"] # EDITED
            #                 length = float(data["distance"]) #length (mi) EDITED
            free_flow_travel_time_ = float(data["travel_time"])
            route_id = data["route_id"]
            l = Link(link_id=len(links),
                     from_node=origin_node, to_node=to_node, flow=float(0.0),
                     free_flow_travel_time=free_flow_travel_time_,
                     link_id_text=id,route_id=route_id)
            links.append(l)

    graph = nx.DiGraph()

    for l in links:
        graph.add_edge(l.from_node, l.to_node, object=l, time=l.free_flow_time)

    return graph


def read_walking_network(Network_file):
    #     walking_network=os.path.join(network_folder,'network_edges.csv')
    #     with open(edges_dir,  "w+",newline='') as csvfile_edges:
    #     writer_edge = csv.writer(csvfile_edges, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    #     writer_edge.writerow(edge_fields)
    with open(Network_file) as f:
        csvreader = csv.DictReader(f)
        nodes = {}
        links = []
        node_list = []

        for data in csvreader:
            origin_node = int(data["from_node"])
            to_node = int(data["to_node"])
            length = float(data["distance"])  # meter
            speed = 1.6  # unit: meter/second
            free_flow_travel_time_ = length / speed  # second
            l = Link(link_id=len(links),
                     from_node=origin_node, to_node=to_node, flow=float(0.0),
                     free_flow_travel_time=free_flow_travel_time_,
                     link_id_text=id)
            links.append(l)
    f.close()
    graph = nx.DiGraph()

    for l in links:
        graph.add_edge(l.from_node, l.to_node, object=l, time=l.free_flow_time)

    return graph


def read_network_dist(Network_file):
    with open(Network_file) as f:
        csvreader = csv.DictReader(f)
        nodes = {}
        links = []
        node_list = []

        for data in csvreader:
            origin_node = int(data["from_node"])
            to_node = int(data["to_node"])
            #                 length = float(data["distance"]) #length (ft)
            free_flow_travel_time_ = float(data["distance"])  # read purely distance
            l = Link(link_id=len(links),
                     from_node=origin_node, to_node=to_node, flow=float(0.0),
                     free_flow_travel_time=free_flow_travel_time_,
                     link_id_text=id)
            links.append(l)

    graph = nx.DiGraph()

    for l in links:
        graph.add_edge(l.from_node, l.to_node, object=l, time=l.free_flow_time)

    return graph


def distance(lat1, lat2, lon1, lon2):
    # The math module contains a function named
    # radians which converts from degrees to radians.
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

    c = 2 * asin(sqrt(a))

    # Radius of earth in kilometers. Use 3956 for miles
    r = 6371

    # calculate the result in meters
    return (c * r) * 1000

def dijsktra_source_to_all(graph, initial,verbose=False):
    transfer_penalty = 120
    visited_temp = {initial: 0}  # dictionary: {node: the weights from source to a visited node}
    path = {}
    try:
        (nodes, edges) = (set(graph.nodes),graph.edges)
        costs = graph.costs
    except: #for NetworkX
        (nodes, edges) = (set(graph.nodes()), graph)  #Siwei: set - sort the graph.nodes
        costs=[]
#     num_nodes=0
    while nodes:
        if verbose == True:
            print("----------------------------------")
        min_node = None
        #find node with minimum distance value, and close this node
        for node in nodes:  #nodes is the list of unexplored nodes
            if node in visited_temp:  #if node is in the list of unexplored nodes and is visited (dist to that "node" is calculated)
                if min_node is None:
                    min_node = node
                    if verbose ==True:
                        print("Selected:", min_node, visited_temp[node])
                elif visited_temp[node] < visited_temp[min_node]:
                    min_node = node
                    if verbose ==True:
                        print("Selected:", min_node, visited_temp[node])
        if min_node is None:
            break
        

        
#         if min_node=='289':
#             verbose=False

        nodes.remove(min_node)  # find the min_node to close, remove the recent close node from the open node list
        permanent = visited_temp[min_node]
        if min_node in edges:
            # get the previous route id's
            if min_node in path.keys():
                prev_node = path[min_node][0]
                prev_route = graph.get_edge_data(prev_node, min_node)["object"].route_id
            else:
                prev_node = None
                prev_route = None
                
                
            for edge in edges[min_node]:
                link_travel_time = graph.get_edge_data(min_node, edge)["object"].free_flow_time #Siwei: get link travel time
                curr_route = graph.get_edge_data(min_node, edge)["object"].route_id
                
                # if the previous node is not the origin,
                # and the route id changed, add a penalty
                if prev_route != None:
                    shared_routes = list(set(prev_route)&set(curr_route))
                    if len(shared_routes) == 0:
                        link_travel_time += transfer_penalty
                
                temp = permanent + link_travel_time #Siwei: temp -> the criteria used for closing a node: travel time for UE

                if edge not in visited_temp or temp < visited_temp[edge]:
                    # KEY POINT
                    visited_temp[edge] = temp
                    path[edge] = (min_node, link_travel_time)

                if verbose ==True:
                     try:
                         print("Permanent:", permanent, "i:", min_node, "j:", edge, "Link cost:",link_travel_time,"Temp Cost:", visited_temp[edge])
                     except:
                         aaa=0
#                 num_nodes+=1
#         print("number of nodes:",num_nodes)

    return visited_temp,path


def dijsktra_all_to_all(graph, verbose=False):
    visited_temp_all = OrderedDict()  # dictionary: {node: source to all shortest path }
    path_all = OrderedDict()
    i = 0
    try:
        (nodes, edges) = (set(graph.nodes), graph.edges)
        costs = graph.costs
    except:  # for NetworkX
        (nodes, edges) = (set(graph.nodes()), graph)  # Siwei: set - sort the graph.nodes
        costs = []
    if verbose == True:
        print("dijsktra_all_to_all starts here", '\n', ':-------------------')
    #     print("nodes",nodes)
    for origin in nodes:
        #         if origin == '172':
        #             verbose=True
        #         else:
        #             verbose=False
        if verbose == True:
            print("node No.", i, "---------")
        visited_temp, path = dijsktra_source_to_all(graph, origin, verbose=verbose)
        #############################################
        # Get the path sequence of the shortest path
        ############################################
        path_all[origin] = OrderedDict()
        for destination in visited_temp.keys():
            if destination != origin:
                if destination not in path_all[origin]:
                    try:
                        path_all[origin][destination] = getTrajectory_O_to_D(origin, destination, path, visited_temp)

                    except Exception as e:
                        print("Shortest path error related to graph connectivity", origin, destination)
                        raise e

        visited_temp = {k: float(v) for k, v in visited_temp.items()} # EDITED
        visited_temp = collections.OrderedDict(sorted(visited_temp.items()))
        visited_temp_all[origin] = visited_temp # EDITED

        i += 1

    return visited_temp_all, path_all


def getTrajectory_O_to_D(origin, destination, path, t_visited, currenttime=0, timeinterval=5.0):
    # try:
    #     j=(destination, (t_visited[destination])/timeinterval,t_visited[destination])  # This line goes wrong.
    # except:
    #     pass
    ###############
    ##Danny's version
    #################
    #     j=(destination)
    #     rtrajectory = [j]
    #     prenode = j[0]

    rtrajectory = []
    prenode = destination
    rtrajectory.append(prenode)
    #     if origin=='172' and destination =='45242':
    #         print("path",path,"\n","\n")
    #         print("t_visited",t_visited)

    # 0317 Siwei
    # path[edge] = (min_node,ts_at_node, link_travel_time, link_travel_marginal_time) # (close node, close time, link cost, link marginal cost)
    for k in range(0, len(path)):

        #         if origin=='172' and destination =='289':
        #             print("prenode",prenode,"\n")
        #             print("rtrajectory",rtrajectory)

        if prenode != origin:
            try:
                nextnode = path[prenode][0]
            except Exception as e:
                print("prenode", prenode, "path", path, "origin", origin, "destination", destination, "t_visited",
                      t_visited)
                raise e
            #             nextnode=path[prenode][0]
            if nextnode == origin:
                rtrajectory.append(path[prenode])
                break
            rtrajectory.append(path[prenode])
            prenode = nextnode

    destination_ = rtrajectory[0]
    rtrajectory_ = []
    pre_node = destination_
    for node, time in rtrajectory[1:]:
        rtrajectory_.append((pre_node, time))
        pre_node = node

    rtrajectory_.append(origin)
    rtrajectory_.reverse()
    rtrajectory_.append(destination)
    # rtrajectory[-1]=(destination, rtrajectory[0][1]+visited[destination])
    return rtrajectory_