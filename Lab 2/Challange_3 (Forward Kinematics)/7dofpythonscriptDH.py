import numpy as np

def dh_transform(theta, d, a, alpha):
    """Calculates the transformation matrix for a single DH row."""
    c_t, s_t = np.cos(theta), np.sin(theta)
    c_a, s_a = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [c_t, -s_t * c_a,  s_t * s_a, a * c_t],
        [s_t,  c_t * c_a, -c_t * s_a, a * s_t],
        [  0,        s_a,        c_a,       d],
        [  0,          0,          0,       1]
    ])

def calculate_7dof_tip(dh_table):
    """Multiplies 7 DH matrices and extracts the final X, Y, Z coordinates."""
    T_final = np.eye(4)
    for row in dh_table:
        T_final = T_final @ dh_transform(*row)
    
    # Extract the X, Y, Z position vector from the 4th column
    tip_position = T_final[:3, 3]
    return np.round(tip_position, 4)

# --- Parameters & Execution ---
if __name__ == "__main__":
    pi_2 = np.pi / 2
    
    # Input your 7 joint angles here (in radians)
    theta = np.deg2rad([0, 30, 0, -45, 0, 60, 0]) 
    
    # Standard DH Table for a 7-DOF arm: [theta, d, a, alpha]
    dh_table_7dof = [
        [theta[0], 0.34, 0.0, -pi_2], # Joint 1
        [theta[1], 0.0,  0.0,  pi_2], # Joint 2
        [theta[2], 0.40, 0.0, -pi_2], # Joint 3
        [theta[3], 0.0,  0.0, -pi_2], # Joint 4
        [theta[4], 0.40, 0.0,  pi_2], # Joint 5
        [theta[5], 0.0,  0.0,  pi_2], # Joint 6
        [theta[6], 0.12, 0.0,   0.0]  # Joint 7 (Tip)
    ]

    tip_xyz = calculate_7dof_tip(dh_table_7dof)
    print(f"7-DOF Tip Position [X, Y, Z]: {tip_xyz}")