import numpy as np

def dh_matrix(theta, d, a, alpha):
    """Calculates the transformation matrix for a single DH row."""
    c_t, s_t = np.cos(theta), np.sin(theta)
    c_a, s_a = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [c_t, -s_t * c_a,  s_t * s_a, a * c_t],
        [s_t,  c_t * c_a, -c_t * s_a, a * s_t],
        [  0,        s_a,        c_a,       d],
        [  0,          0,          0,       1]
    ])

def calculate_6dof_tip(dh_table):
    """Multiplies 6 DH matrices and extracts the final X, Y, Z coordinates."""
    T_final = np.eye(4)
    for row in dh_table:
        T_final = T_final @ dh_matrix(*row)
    
    # Extract the X, Y, Z position vector from the 4th column
    tip_position = T_final[:3, 3]
    return np.round(tip_position, 4)

# --- Parameters & Execution ---
if __name__ == "__main__":
    pi_2 = np.pi / 2
    
    # Input your 6 joint angles here (in radians)
    theta = np.deg2rad([0, -90, 0, -90, 0, 0]) 
    
    # Standard DH Table for a 6-DOF arm: [theta, d, a, alpha]
    dh_table_6dof = [
        [theta[0], 0.089159,  0.0,       pi_2], # Joint 1
        [theta[1], 0.0,      -0.42500,    0.0], # Joint 2
        [theta[2], 0.0,      -0.39225,    0.0], # Joint 3
        [theta[3], 0.10915,   0.0,       pi_2], # Joint 4
        [theta[4], 0.09465,   0.0,      -pi_2], # Joint 5
        [theta[5], 0.0823,    0.0,        0.0]  # Joint 6 (Tip)
    ]

    tip_xyz = calculate_6dof_tip(dh_table_6dof)
    print(f"6-DOF Tip Position [X, Y, Z]: {tip_xyz}")