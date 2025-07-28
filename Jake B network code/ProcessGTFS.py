import geopy.distance
import numpy as np
import math
import pandas as pd

class GTFSNetwork:
    
    def to_int_id(self, str_id):
        '''
        Converts string id to int
        Adds id pair to int_ids{}        
        
        '''
        
        if type(str_id) == str and not str_id.isnumeric():
            int_id = hash(str_id)%100000
        else:
            int_id = int(str_id)
        
        self.int_ids[int_id] = str_id
        
        return int_id
    
    def to_str_id(self, int_id):
        
        return str(self.int_ids[int_id])
            
    
    def __init__(self,stop_times_fn,stops_fn,trips_fn, routes_fn):
        # Constants
        self.speeds = [30]          # If possible, should be a list of speeds in mph for each route
        self.walking_speed = 2.8    # mph
        
        # Dict for converting int id's back into str id's
        self.int_ids = {}
        
        self.s_hash = {}
        
        self.stop_times_df = pd.read_csv(stop_times_fn, low_memory=False)
        self.stop_times_df = self.stop_times_df[['trip_id','stop_id','stop_sequence',
                               'shape_dist_traveled']]
        
        self.stops_df = pd.read_csv(stops_fn)
        self.stops_df = self.stops_df[['stop_id','stop_lat','stop_lon']]
        
        self.trips_df = pd.read_csv(trips_fn)
        self.trips_df = self.trips_df[['route_id','trip_id']]
        
        self.routes_df = pd.read_csv(routes_fn)
        self.routes_df = self.routes_df[['route_id', 'route_type']]
        
        # Convert str to int id     
        if type(self.stop_times_df.iloc[0]['trip_id']) == str:
            self.stop_times_df['trip_id'] = self.stop_times_df['trip_id'].apply(self.to_int_id)
        if type(self.stop_times_df.iloc[0]['stop_id']) == str:
            self.stop_times_df['stop_id'] = self.stop_times_df['stop_id'].apply(self.to_int_id)
        

        if type(self.stops_df.iloc[0]['stop_id']) == str:
            str_id = self.stops_df['stop_id']
            self.stops_df['stop_id'] = self.stops_df['stop_id'].apply(self.to_int_id)
            

        if type(self.trips_df.iloc[0]['route_id']) == str:
            self.trips_df['route_id'] = self.trips_df['route_id'].apply(self.to_int_id)
        if type(self.trips_df.iloc[0]['trip_id']) == str:
            self.trips_df['trip_id'] = self.trips_df['trip_id'].apply(self.to_int_id)

            
        if type(self.routes_df.iloc[0]['route_id']) == str:
            self.routes_df['route_id'] = self.routes_df['route_id'].apply(self.to_int_id)    
        
        # Create necessary dicts
        self.route_trips = {}
        self.route_stops = {}
        self.stop_routes = {}
        self.generate_route_trips()
        self.generate_route_stops()
        self.generate_stop_routes()
        

    def generate_network_df(self):
        network_df = pd.DataFrame(columns=['from_node', 'to_node','distance',
                                       'travel_time','link_type','route_id', 'route_type'])

        # Add all connections for nodes adjacent on the same route
        for i in range(0,len(self.stop_times_df)-1):
            currRow = self.stop_times_df.iloc[i].to_numpy()
            nextRow = self.stop_times_df.iloc[i+1].to_numpy()
            from_node_id = int(currRow[1])
            to_node_id = int(nextRow[1])
            # Check for matching trip_id
            # and correct sequence (may be redundant)
            if not (currRow[0] == nextRow[0] and currRow[2] + 1 == nextRow[2]):
                continue
            # Check if this pair of stops is alrady in network_df
            if network_df.loc[network_df['from_node']==from_node_id]['to_node'].isin([to_node_id]).any():
                continue

            dist = 0.621371*(nextRow[3] - currRow[3])  # convert distances km to mi
            time = (dist / self.speeds[0]) * 3600      # time in sec

            # Get the route_id(s) that both stops share
            route_id = list(set(self.stop_routes[from_node_id])&set(self.stop_routes[to_node_id]))
            
            route_type = self.routes_df.loc[self.routes_df['route_id']==route_id[0]]['route_type'].to_numpy()[0]

            # Add this pair to the network
            data = {'from_node':from_node_id,
                   'to_node':to_node_id,
                   'distance':dist,
                   'travel_time':time,
                   'link_type':0,
                   'route_id':route_id,
                   'route_type':int(route_type)}

            network_df.loc[len(network_df)] = data

            # Add reverse pair to the network
            data = {'from_node':to_node_id,
                   'to_node':from_node_id,
                   'distance':dist,
                   'travel_time':time,
                   'link_type':0,
                   'route_id':route_id,
                   'route_type':int(route_type)}

            network_df.loc[len(network_df)] = data
            
        # Add all transfer links
        # Iterate through all stops in stop_routes
        # Check all nearby stops
        # Add pairs for all stops within thresh distance
        for i in range(0,len(self.stops_df)):
            # Get all stops which should be linked
            stop = self.stops_df.iloc[i].to_numpy()
            linked_stops = self.s_hash.get_linked_stops(stop[0],stop[1],stop[2],self.stop_routes)
            
            # Add edges for all linked stops
            for linked in linked_stops:
                dist = linked_stops[linked]
                time = (dist / self.walking_speed) * 3600
                
                from_node_id = int(stop[0])
                to_node_id = int(float(linked))
                route_id = -1
                
                # Add this pair to the network
                data = {'from_node':from_node_id,
                       'to_node':to_node_id,
                       'distance':dist,
                       'travel_time':time,
                       'link_type':1,
                       'route_id':route_id,
                       'route_type':-1}
                network_df.loc[len(network_df)] = data
                
                # Add reverse pair to the network
                data = {'from_node':to_node_id,
                       'to_node':from_node_id,
                       'distance':dist,
                       'travel_time':time,
                       'link_type':1,
                       'route_id':route_id,
                       'route_type':-1}
                network_df.loc[len(network_df)] = data
        
        return network_df
    
    def generate_node_file(self, xlsx_fn, txt_fn):
        node_file_df = self.stops_df
        node_file_df['string_id'] = node_file_df['stop_id'].apply(self.to_str_id)
        self.stops_df.to_excel(xlsx_fn)
        self.stops_df.to_csv(txt_fn)

    def generate_route_trips(self):
        '''
        Creates dict route_trips
            Key:   route_id
            Value: np.array of trip_id's
        '''
        route_ids = np.unique(self.trips_df[['route_id']].to_numpy())
        self.route_trips = {}

        # Add all trip_id for each route_id
        for route_id in route_ids:
            trips = self.trips_df.loc[self.trips_df['route_id']==route_id]['trip_id'].to_numpy()
            self.route_trips[route_id] = np.unique(trips)

    def generate_route_stops(self):
        '''
        Creates dict route_stops
            Key:   route_id
            Value: np.array of stop_id's
        '''

        # Create route_stops
        self.route_stops = {}
        for route_id in self.route_trips:
            # Create lists of stops for each route_id
            # Get all stop_id's for each trip_id
            trip_stops = np.array([])
            for trip_id in self.route_trips[route_id]:
                stops = self.stop_times_df.loc[self.stop_times_df['trip_id']==trip_id]
                stops = stops[['stop_id']].to_numpy()
                trip_stops = np.append(trip_stops, stops)
            # Add all stop_id's for the given route_id
            self.route_stops[route_id] = np.unique(trip_stops)

    def generate_stop_routes(self):
        '''
        Creates dict stop_routes
            Key:   stop_id
            Value: route_id
        '''
        # FIXME all stops only have one route_id
        # That doesn't seem correct
        # Also, 104 stops are missing

        self.stop_routes = {}
        for route_id in self.route_stops:
            for stop in self.route_stops[route_id]:
                if not stop in self.stop_routes.keys():
                    self.stop_routes[stop] = np.array([route_id])
                else:
                    self.stop_routes[stop] = np.append(self.stop_routes[stop], route_id)
        

class S_Hash:
    def __init__(self,north,south,east,west,thresh):
        '''
        Sets latitude and longitude limits
        Creates number of buckets and step size based on thresh
        '''
        self.north = north
        self.south = south
        self.east = east
        self.west = west
        self.thresh = thresh
        
        height = geopy.distance.geodesic((north,west), (south,west)).mi
        width = geopy.distance.geodesic((north,west),(north,east)).mi

        
        self.num_buckets = int(math.ceil(max(height,width)) / thresh)

        # Determine step size for lat and lon
        self.step = max((abs(north - south) / self.num_buckets,
                    abs(west - east) / self.num_buckets))
        
        # Bucket index : (coordinates in that bucket, stop_id of coordinates)
        self.buckets = {}
    
    def get_key(self, lat, lon):
        '''
        Returns the bucket key for a set of coordinates
        '''
        lon_dist = abs(self.west - lon)
        lat_steps = math.floor(lon_dist/self.step)
        lat_dist = abs(self.north - lat)
        lon_steps = math.floor(lat_dist/self.step)
        return (lat_steps,lon_steps)
    
    def generate_hash(self, stops_df):
        '''
        Populates the hash with stops in stops_df
        '''
        # Add all nodes to the hash
        for i in range(len(stops_df)):
            stop = stops_df.iloc[i]
            stop_id = stop[0]
            lat = stop[1]
            lon = stop[2]
            key = self.get_key(lat,lon)
            if not key in self.buckets.keys():
                self.buckets[key] = np.vstack((['0',0,0],(stop_id, lat, lon)))
                # FIXME get around issue of 1 coordinate in a box, not very elegant
            else:
                self.buckets[key] = np.vstack((self.buckets[key],
                                               (stop_id, lat, lon)))
    
    def get_nearby_stops(self, lat, lon):
        '''
        Returns dict stops of stops within self.thresh distance of lat,lon
            Key:   stop_id
            Value: dist to given lat, lon in mi
        '''
        stops = {}
        coords1 = (lat,lon)
        key = self.get_key(lat,lon)
        
        # Check coordinates in the same bucket
        for stop in self.buckets[key]:
            stop_id = stop[0]
            coords2 = (stop[1],stop[2])
            dist = geopy.distance.geodesic(coords1,coords2).mi
            if 0 < dist < self.thresh:
                stops[stop_id] = dist
        
        # Check coordinates in neighboring buckets
        # FIXME is there a way to do this more elegantly
        for i in range(-1,2,2):
            neigh_key = (key[0]+i,key[1])
            if neigh_key in self.buckets.keys():
                for stop in self.buckets[neigh_key]:
                    stop_id = stop[0]
                    coords2 = (stop[1],stop[2])
                    dist = geopy.distance.geodesic(coords1,coords2).mi
                    if 0 < dist < self.thresh:
                        stops[stop_id] = dist
            neigh_key = (key[0],key[1]+i)
            if neigh_key in self.buckets.keys():
                for stop in self.buckets[neigh_key]:
                    stop_id = stop[0]
                    coords2 = (stop[1],stop[2])
                    dist = geopy.distance.geodesic(coords1,coords2).mi
                    if 0 < dist < self.thresh:
                        stops[stop_id] = dist
            neigh_key = (key[0]+i,key[1]+i)
            if neigh_key in self.buckets.keys():
                for stop in self.buckets[neigh_key]:
                    stop_id = stop[0]
                    coords2 = (stop[1],stop[2])
                    dist = geopy.distance.geodesic(coords1,coords2).mi
                    if 0 < dist < self.thresh:
                        stops[stop_id] = dist
        neigh_key = (key[0]+1,key[1]-1)
        if neigh_key in self.buckets.keys():
            for stop in self.buckets[neigh_key]:
                stop_id = stop[0]
                coords2 = (stop[1],stop[2])
                dist = geopy.distance.geodesic(coords1,coords2).mi
                if 0 < dist < self.thresh:
                    stops[stop_id] = dist
        neigh_key = (key[0]-1,key[1]+1)
        if neigh_key in self.buckets.keys():
            for stop in self.buckets[neigh_key]:
                stop_id = stop[0]
                coords2 = (stop[1],stop[2])
                dist = geopy.distance.geodesic(coords1,coords2).mi
                if 0 < dist < self.thresh:
                    stops[stop_id] = dist
        return stops
    
    def get_linked_stops(self, stop_id, lat, lon, stop_routes):
        '''
        Returns dict of stop_id:dist within thresh distance
        of lat and lon and which do not share a route
        dist in mi
        '''
        nearby_stops = self.get_nearby_stops(lat,lon)
        linked_stops = {}
        if not stop_id in stop_routes.keys():
            return nearby_stops
        routes1 = stop_routes[stop_id]
        for nearby_stop in nearby_stops:
            if nearby_stop not in stop_routes.keys():
                continue
            routes2 = stop_routes[nearby_stop]
            # Only add if stops don't share a route
            if len(list(set(routes1)&set(routes2))) == 0:
                linked_stops[nearby_stop] = nearby_stops[nearby_stop]
        return linked_stops