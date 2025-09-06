#!/usr/bin/env python3
"""
Simplified Graph-Based Trust Integration for VehicleProcess.py

This file contains the minimal trust evaluation methods needed for fleet graph-based
trust evaluation. These methods replace the complex trust evaluation system.
"""

from typing import Optional, Dict, List, Any
import time
import math
import numpy as np

class GraphBasedTrustEvaluator:
    """
    Simplified trust evaluator for graph-connected vehicles.
    Uses TriPTrustModel only for vehicles that are connected in the fleet graph.
    """
    
    def __init__(self, vehicle_id: int, connected_vehicles: List[int], trust_models: Dict):
        self.vehicle_id = vehicle_id
        self.connected_vehicles = connected_vehicles
        self.trust_models = trust_models
        self.trust_scores = {}
        
        # Initialize trust scores for connected vehicles
        for vehicle_id in connected_vehicles:
            if vehicle_id != self.vehicle_id:
                self.trust_scores[vehicle_id] = 1.0  # Start with maximum trust
    
    def evaluate_trust_for_received_state(self, sender_id: int, received_state: dict, 
                                        our_state: dict, logger=None) -> Optional[float]:
        """
        Evaluate trust when receiving state from a connected vehicle.
        
        Args:
            sender_id: ID of sender vehicle
            received_state: Received state data
            our_state: Our own vehicle state
            logger: Logger for output
            
        Returns:
            Trust score (0.0 to 1.0) or None if not applicable
        """
        # Only evaluate for connected vehicles
        if sender_id not in self.connected_vehicles or sender_id not in self.trust_models:
            return None
            
        try:
            trust_model = self.trust_models[sender_id]
            
            # Extract state information
            reported_velocity = received_state.get('velocity', 0.0)
            reported_position = received_state.get('position', [0, 0, 0])
            reported_acceleration = received_state.get('acceleration', 0.0)
            
            our_velocity = our_state.get('velocity', 0.0)
            our_position = our_state.get('position', [0, 0, 0])
            our_acceleration = our_state.get('acceleration', 0.0)
            
            # Calculate distance
            if len(reported_position) >= 2 and len(our_position) >= 2:
                distance = math.sqrt((reported_position[0] - our_position[0])**2 + 
                                   (reported_position[1] - our_position[1])**2)
            else:
                distance = 10.0

            is_nearby = abs(sender_id - self.vehicle_id) == 1 

            # Evaluate trust components using TriPTrustModel
            v_score = trust_model.evaluate_velocity(
                host_id=self.vehicle_id,
                target_id=sender_id,
                v_y=reported_velocity,
                v_host=our_velocity,
                v_leader=our_velocity,  # Simplified
                a_leader=our_acceleration,
                b_leader=0.1,
                is_nearby=is_nearby
            )
            
            d_score = trust_model.evaluate_distance(
                d_y=distance,
                d_measured=distance,
                is_nearby=is_nearby
            )
            
            # Simplified acceleration and other scores
            a_score = 1.0 if abs(reported_acceleration) < 5.0 else 0.5
            beacon_score = 1.0  # We received the message
            h_score = 1.0  # Simplified heading
            
            # Calculate trust sample
            trust_sample = trust_model.calculate_trust_sample(
                v_score=v_score,
                d_score=d_score,
                a_score=a_score,
                beacon_score=beacon_score,
                h_score=h_score,
                is_nearby=is_nearby
            )
            
            # Update rating vector
            trust_model.update_rating_vector(trust_sample, vector_type="local")
            
            # Calculate final trust score
            final_trust_score = trust_model.calculate_trust_score(trust_model.rating_vector)
            
            # Update our stored trust score
            self.trust_scores[sender_id] = final_trust_score
            
            logger.info(f"TRUST_SCORE: Vehicle {sender_id} -> {final_trust_score:.3f} "
                    f"(v_score={v_score:.3f}, d_score={d_score:.3f}, distance={distance:.1f}m)")
           
            return final_trust_score
            
        except Exception as e:
            if logger:
                logger.error(f"TRUST_EVAL: Error evaluating trust for vehicle {sender_id}: {e}")
            return None
    
    def get_trust_score(self, vehicle_id: int) -> float:
        """Get current trust score for a vehicle."""
        return self.trust_scores.get(vehicle_id, 1.0)
    
    def get_all_trust_scores(self) -> Dict[int, float]:
        """Get all current trust scores."""
        return self.trust_scores.copy()
    
    def is_vehicle_trusted(self, vehicle_id: int, threshold: float = 0.5) -> bool:
        """Check if a vehicle is trusted above threshold."""
        return self.get_trust_score(vehicle_id) >= threshold
    
    def apply_trust_based_control(self, following_target: int, base_distance: float, logger=None) -> float:
        """
        Apply trust-based control adjustments.
        
        Args:
            following_target: ID of vehicle we're following
            base_distance: Base following distance
            logger: Logger for output
            
        Returns:
            Adjusted following distance
        """
        if following_target not in self.connected_vehicles:
            return base_distance
            
        trust_score = self.get_trust_score(following_target)
        
        # Adjust distance based on trust (lower trust = larger distance)
        trust_factor = max(0.5, trust_score)  # Minimum 50% trust
        adjusted_distance = base_distance / trust_factor
        
        # Log significant adjustments
        if abs(adjusted_distance - base_distance) > 1.0 and logger:
            logger.info(f"TRUST_CONTROL: Distance adjustment for vehicle {following_target} - "
                       f"base={base_distance:.1f}m -> adjusted={adjusted_distance:.1f}m "
                       f"(trust={trust_score:.3f})")
        
        return adjusted_distance

    def log_trust_summary(self, logger=None):
        """
        Log a summary of all current trust scores for monitoring.
        
        Args:
            logger: Logger for output
        """
        if logger and len(self.trust_scores) > 0:
            trust_summary = []
            for vehicle_id, score in self.trust_scores.items():
                trust_summary.append(f"V{vehicle_id}={score:.3f}")
            
            logger.info(f"TRUST_SUMMARY: Connected vehicles trust scores: {', '.join(trust_summary)}")
    
    def get_trust_statistics(self) -> Dict[str, float]:
        """
        Get statistical information about current trust scores.
        
        Returns:
            Dictionary with trust statistics
        """
        if not self.trust_scores:
            return {'count': 0, 'mean': 0.0, 'min': 0.0, 'max': 0.0}
        
        scores = list(self.trust_scores.values())
        return {
            'count': len(scores),
            'mean': sum(scores) / len(scores),
            'min': min(scores),
            'max': max(scores)
        }


# Example integration helper methods for VehicleProcess
def integrate_graph_based_trust_into_vehicle_process():
    """
    Instructions for integrating this simplified trust system into VehicleProcess.py
    
    1. In __init__ method, replace complex trust initialization with:
    
        if self.trust_enabled and len(self.connected_vehicles) > 0:
            self.trust_evaluator = GraphBasedTrustEvaluator(
                vehicle_id=self.vehicle_id,
                connected_vehicles=self.connected_vehicles,
                trust_models={vid: TriPTrustModel() for vid in self.connected_vehicles if vid != self.vehicle_id}
            )
        else:
            self.trust_evaluator = None
    
    2. In _evaluate_trust_for_received_state method, replace complex evaluation with:
    
        if hasattr(self, 'trust_evaluator') and self.trust_evaluator is not None:
            our_state = self.get_best_available_state()
            trust_score = self.trust_evaluator.evaluate_trust_for_received_state(
                sender_id, received_state, our_state, self.logger
            )
            if trust_score is not None and self.following_target == sender_id:
                base_distance = self.config.get('distance_between_cars', 8.0)
                adjusted_distance = self.trust_evaluator.apply_trust_based_control(
                    sender_id, base_distance, self.logger
                )
    
    3. Replace other trust methods with simple delegations:
    
        def get_trust_score(self, vehicle_id):
            if hasattr(self, 'trust_evaluator') and self.trust_evaluator:
                return self.trust_evaluator.get_trust_score(vehicle_id)
            return 1.0
    
        def is_vehicle_trusted(self, vehicle_id, threshold=0.5):
            if hasattr(self, 'trust_evaluator') and self.trust_evaluator:
                return self.trust_evaluator.is_vehicle_trusted(vehicle_id, threshold)
            return True
    """
    pass

if __name__ == "__main__":
    print("Graph-Based Trust Evaluator created")
    print("See integration instructions in the code")
