import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb

def plot_franka_workspace(num_samples=1000000):
    print("Loading Franka Panda model...")
    # Load the official built-in model for Franka Emika Panda
    franka = rtb.models.URDF.Panda()
    
    # Generate random joint configurations within physical joint limits
    q_rand = np.random.uniform(franka.qlim[0, :], franka.qlim[1, :], (num_samples, franka.n))
    
    # Calculate Forward Kinematics for all samples simultaneously 
    T = franka.fkine(q_rand)
    
    # Extract end-effector X, Y, Z coordinates
    x, y, z = T.t[:, 0], T.t[:, 1], T.t[:, 2]
    
    # Render Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='teal', s=1, alpha=0.5)
    
    ax.set_title("Franka Emika Panda - Reachable Workspace")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")
    plt.show()

def plot_heal_workspace(num_samples=1000000):
    print("Building HEAL generic kinematic model...")
    
    # 7-DOF MDH (Modified Denavit-Hartenberg) Placeholder Template for HEAL
    # IMPORTANT: Replace the 'd' (link offset) and 'a' (link length) parameters 
    # with the official measurements from the HEAL datasheet.
    heal = rtb.DHRobot([
        rtb.RevoluteMDH(d=0.333, a=0,     alpha=0,        qlim=[-2.9, 2.9]),
        rtb.RevoluteMDH(d=0,     a=0,     alpha=-np.pi/2, qlim=[-1.8, 1.8]),
        rtb.RevoluteMDH(d=0.316, a=0,     alpha=np.pi/2,  qlim=[-2.9, 2.9]),
        rtb.RevoluteMDH(d=0,     a=0.082, alpha=np.pi/2,  qlim=[-3.1, 0.0]),
        rtb.RevoluteMDH(d=0.384, a=-0.082,alpha=-np.pi/2, qlim=[-2.9, 2.9]),
        rtb.RevoluteMDH(d=0,     a=0,     alpha=np.pi/2,  qlim=[-3.8, 2.9]),
        rtb.RevoluteMDH(d=0.107, a=0.088, alpha=np.pi/2,  qlim=[-2.9, 2.9])
    ], name="HEAL 7-DOF Cobot")
    
    # Generate random joint configurations within the defined limits
    q_rand = np.random.uniform(heal.qlim[0, :], heal.qlim[1, :], (num_samples, heal.n))
    
    # Calculate Forward Kinematics
    T = heal.fkine(q_rand)
    
    x, y, z = T.t[:, 0], T.t[:, 1], T.t[:, 2]
    
    # Render Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c='coral', s=1, alpha=0.5)
    
    ax.set_title("HEAL Cobot - Reachable Workspace")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")
    plt.show()

if __name__ == "__main__":
    # Run the visualization tools
    plot_franka_workspace()
    plot_heal_workspace()