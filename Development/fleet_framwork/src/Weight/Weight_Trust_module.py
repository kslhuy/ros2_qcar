import numpy as np


class WeightTrustModule:
    """
    TrustManager - A class to manage trust scores, trusted neighbors, and weights
    for a platoon of vehicles under potential attacks.
    """
    
    def __init__(self, graph, trust_threshold, kappa):
        """
        Initialize the TrustManager with graph and parameters
        
        Parameters:
            graph              - Adjacency matrix (n x n) as numpy array
            trust_threshold    - Trust score threshold (e.g., 0.5)
            kappa              - Influence limit parameter
        """
        self.graph = np.array(graph)
        self.trust_threshold = trust_threshold
        self.kappa = kappa
        self.num_vehicles = self.graph.shape[0]
    
    def get_trusted_neighbors(self, car_idx, trust_scores):
        """
        Compute trusted neighbors for each vehicle based on current trust scores
        
        Parameters:
            car_idx      - Index of the vehicle (0-based)
            trust_scores - Array or list of trust scores for all vehicles
            
        Returns:
            neighbors    - Array of indices of trusted neighbors of the vehicle
        """
        # Convert trust_scores to numpy array if it's not already
        trust_scores_array = np.array(trust_scores)
        
        # Find direct neighbors (non-zero entries in adjacency matrix)
        neighbors = np.where(self.graph[car_idx, :])[0]
        
        # Remove neighbors whose trust score is below the threshold
        trusted_mask = trust_scores_array[neighbors] > self.trust_threshold
        trusted_neighbors = neighbors[trusted_mask]
        
        return trusted_neighbors
    
    def calculate_weights_trust(self, vehicle_index, trust_scores, weight_type="local"):
        """
        Compute weight matrix based on trusted neighbors
        
        Parameters:
            vehicle_index - Index of the vehicle (0-based)
            trust_scores  - Array of trust scores for all vehicles
            weight_type   - Type of weighting ("local" or "distributed")
            
        Returns:
            weights_dis   - Weight array (1 x num_vehicles + 1)
        """
        virtual_graph = self.generate_virtual_graph(self.graph, vehicle_index)
        trusted_neighbors_set = self.get_trusted_neighbors(vehicle_index, trust_scores)
        num_nodes = virtual_graph.shape[0]
        weights_dis = np.zeros((1, num_nodes))
        
        N_i_t = trusted_neighbors_set  # Trusted neighbors (0-based indices)
        n_w_i = max(self.kappa, len(N_i_t) + 1 + 1)  # +1 for self, +1 for virtual node
        weight = 1.0 / n_w_i
        
        # Set self-weight: weights_dis[0, vehicle_index+1]
        weights_dis[0, vehicle_index + 1] = weight
        
        # Set weights for trusted neighbors: weights_dis[0, l+1]
        for l in N_i_t:
            weights_dis[0, l + 1] = weight
        
        # Set weight to virtual node: weights_dis[0, 0]
        if weight_type == "local":
            # Using weight matrix prioritizing local estimation
            weights_dis[0, 0] = 1 - (len(N_i_t) + 1) * weight
        else:
            # Using weight matrix distributed equally
            weights_dis[0, 0] = weight
            
        # NOTE: Either using local or distributed weight matrix
        # One problem is the weight matrix weights_dis[0, 0] still needs to be adjusted based on the trust
        # See more details in Observer class
        
        return weights_dis
    
    def calculate_weights_default(self, vehicle_index):
        """
        Function to calculate consensus weights for a given virtual graph
        
        Parameters:
            vehicle_index - Index of the vehicle (0-based)
            
        Returns:
            weights_dis   - Weight array (1 x num_vehicles + 1)
        """
        Vj = self.generate_virtual_graph(self.graph, vehicle_index)
        num_nodes = Vj.shape[0]
        W = np.zeros((num_nodes, num_nodes))  # Initialize weights matrix
        
        # Start from index 1 (second node) since index 0 is the virtual node
        for i in range(1, num_nodes):
            d_i = np.sum(Vj[i, :])  # Degree of node i
            for l in range(num_nodes):  # Include all nodes in the virtual graph
                if Vj[i, l] == 1 or i == l:  # Neighbor or self
                    W[i, l] = 1.0 / (d_i + 1)
        
        # Return the row corresponding to the vehicle (convert to 0-based indexing)
        weights_dis = W[vehicle_index + 1, :].reshape(1, -1)  # Shape: (1, num_vehicles + 1)
        
        return weights_dis
    
    def generate_virtual_graph(self, graph, vehicle_index):
        """
        Generate a virtual graph by adding an extra node (node 0) and connecting it
        to the specified vehicle and its neighbors.
        
        Parameters:
            graph         - Adjacency matrix of the original graph
            vehicle_index - Index of the vehicle to which node 0 will be connected (0-based)
            
        Returns:
            Vj           - Virtual graph adjacency matrix with an extra node
        """
        # Initialize virtual graph with an extra node
        Vj = np.zeros((self.num_vehicles + 1, self.num_vehicles + 1))
        
        # Copy the adjacency matrix to the virtual graph (shift by 1 to account for virtual node)
        Vj[1:, 1:] = graph
        
        # Set the bidirectional edge between node 0 and the specified vehicle
        Vj[0, vehicle_index + 1] = 1
        Vj[vehicle_index + 1, 0] = 1
        
        # Set the edges between node 0 and vehicle's neighbors
        neighbors = np.where(graph[vehicle_index, :])[0]
        for neighbor in neighbors:
            Vj[0, neighbor + 1] = graph[vehicle_index, neighbor]
            Vj[neighbor + 1, 0] = graph[neighbor, vehicle_index]
        
        return Vj
