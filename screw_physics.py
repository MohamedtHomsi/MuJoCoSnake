#!/usr/bin/env python3
"""
Standalone physics model for screw-based locomotion.
Based on equations from equations.txt
"""

import numpy as np

class ScrewPhysics:
    """Physics model based on equations.txt"""
    def __init__(self, h_b=0.055, D_d=0.10, mu=0.7, alpha_m=0.0, W=5.0):
        """
        Parameters:
        h_b: blade height (m) - mean height of helix blade above base drum
        D_d: drum diameter (m) - base drum diameter
        mu: coefficient of friction between screw and ground
        alpha_m: slope angle (radians) - terrain slope
        W: weight per screw segment (N)
        """
        self.h_b = h_b
        self.D_d = D_d
        self.mu = mu
        self.alpha_m = alpha_m
        self.W = W
        
        # Calculate helix angle from geometry
        # k = h_b / D_d
        # tan(θ) = h / (π * k * D_d)
        # For simplicity, assume h ≈ h_b (one pitch height)
        k = h_b / D_d
        self.theta = np.arctan(h_b / (np.pi * k * D_d))
        
        print(f"Screw Physics Initialized:")
        print(f"  Blade height h_b = {h_b:.4f} m")
        print(f"  Drum diameter D_d = {D_d:.4f} m")
        print(f"  Coefficient of friction μ = {mu:.3f}")
        print(f"  Helix angle θ = {np.degrees(self.theta):.2f}°")
        print(f"  Weight per screw W = {W:.2f} N")
    
    def calculate_axial_force(self):
        """Calculate axial force F from equation 7 & 19"""
        # F = W (cos(α_m) + μ sin(α_m)) / (cos(θ) - μ sin(θ))
        numerator = self.W * (np.cos(self.alpha_m) + self.mu * np.sin(self.alpha_m))
        denominator = np.cos(self.theta) - self.mu * np.sin(self.theta)
        
        if abs(denominator) < 1e-6:
            print("Warning: denominator near zero in force calculation")
            return 0.0
        
        F = numerator / denominator
        return F
    
    def calculate_required_torque(self, v, n):
        """
        Calculate required torque based on power equations (equations 24-34, 47)
        
        Parameters:
        v: forward velocity (m/s)
        n: rotation speed (rev/s)
        
        Returns:
        T: required torque (N⋅m)
        """
        if abs(v) < 1e-6 or abs(n) < 1e-6:
            return 0.0
        
        F = self.calculate_axial_force()
        omega = 2 * np.pi * n  # angular velocity (rad/s)
        
        # Power losses:
        # a) Drum friction (equation 24)
        # P_drum = (μW cos(θ) / sin(θ)) * π * D_d * n_d
        # where n_d = n (assuming drum rotates at same speed)
        if abs(np.sin(self.theta)) > 1e-6:
            P_drum = (self.mu * self.W * np.cos(self.theta) / np.sin(self.theta)) * np.pi * self.D_d * n
        else:
            P_drum = 0.0
        
        # b) Helix blade friction (equation 28)
        # P_blade = (μF) * π * (D_d + h_b) * n / cos(θ)
        if abs(np.cos(self.theta)) > 1e-6:
            P_blade = (self.mu * F) * np.pi * (self.D_d + self.h_b) * n / np.cos(self.theta)
        else:
            P_blade = 0.0
        
        # c) Work against gravity (equation 31)
        # P_w = W * v * sin(θ)
        P_w = self.W * v * np.sin(self.theta)
        
        # Total power (equation 34)
        P_total = P_drum + P_blade + P_w
        
        # Torque from power (equation 42, 47)
        # T = P / ω
        if abs(omega) > 1e-6:
            T = P_total / omega
        else:
            T = 0.0
        
        return T
    
    def get_power_breakdown(self, v, n):
        """
        Get detailed power breakdown for analysis
        
        Returns:
        dict with P_drum, P_blade, P_w, P_total
        """
        if abs(v) < 1e-6 or abs(n) < 1e-6:
            return {
                'P_drum': 0.0,
                'P_blade': 0.0,
                'P_w': 0.0,
                'P_total': 0.0,
                'gamma': 0.0
            }
        
        F = self.calculate_axial_force()
        
        # Calculate individual power components
        if abs(np.sin(self.theta)) > 1e-6:
            P_drum = (self.mu * self.W * np.cos(self.theta) / np.sin(self.theta)) * np.pi * self.D_d * n
        else:
            P_drum = 0.0
        
        if abs(np.cos(self.theta)) > 1e-6:
            P_blade = (self.mu * F) * np.pi * (self.D_d + self.h_b) * n / np.cos(self.theta)
        else:
            P_blade = 0.0
        
        P_w = self.W * v * np.sin(self.theta)
        P_total = P_drum + P_blade + P_w
        
        # Non-dimensional power (equation 38)
        gamma = P_total / (self.W * v) if abs(v) > 1e-6 else 0.0
        
        return {
            'P_drum': P_drum,
            'P_blade': P_blade,
            'P_w': P_w,
            'P_total': P_total,
            'gamma': gamma
        }

