import numpy as np

# Plotting Libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

# Custom libraries
import hardware

hrd = hardware.hardware()

# Initialize the fiugre
fig1 = plt.figure(1)
ax = fig1.add_subplot( 111, projection = '3d' )
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
fig1.suptitle('Foot Positions, Body Frame')
fig1.tight_layout()
fig1.show()

# Plot the hexagon shoulder joint position
s = np.array( [ hrd.s[:,0], hrd.s[:,1], hrd.s[:,3], hrd.s[:,5], hrd.s[:,4], hrd.s[:,2], hrd.s[:,0] ] ).T
ax.plot3D( s[0,:], s[1,:], s[2,:], label="Body", color='k', linewidth=4)
ax.plot3D( s[0,:], s[1,:], 0, color='k', linewidth=4)

# Set plotting limits
xrange = [-0.35, 0.35]
yrange = [-0.35, 0.35]
zrange = [-0.031, 0.05]

# Loop through all joint angles, calculate possible foot positions
for leg in range(6):

    foot_position = hrd.s[:,leg]
    for q1 in np.linspace( hrd.motorMin[0,leg], hrd.motorMax[0,leg], 20 ):
        q1 = ( q1 - hrd.motorCenter[0,leg] ) * np.pi/180 * hrd.motorOrientation[0, leg]

        for q2 in np.linspace( hrd.motorMin[1,leg], hrd.motorMax[1,leg], 20 ):
            q2 = ( q2 - hrd.motorCenter[1,leg] ) * np.pi/180 * hrd.motorOrientation[1, leg]

            for q3 in np.linspace( hrd.motorMin[2,leg], hrd.motorMax[2,leg], 20 ):
                q3 = ( q3 - hrd.motorCenter[2,leg] ) * np.pi/180 * hrd.motorOrientation[2, leg]

                joint_angles = [q1, q2, q3]

                hrd.joint_angles[:,leg] = joint_angles
                T_b2f = hrd.fkine_body2foot( leg )

                foot_pos = T_b2f[0:3,3]
                if foot_pos[0] > xrange[0] and foot_pos[0] < xrange[1] and \
                    foot_pos[1] > yrange[0] and foot_pos[1] < yrange[1] and \
                    foot_pos[2] > zrange[0] and foot_pos[2] < zrange[1]:

                    foot_position = np.vstack( [ foot_position, foot_pos ] )

    ax.scatter( foot_position[:,0], foot_position[:,1], foot_position[:,2], label=str('Leg'+str(leg+1)), marker='.' )

# Plot foot movement circles
theta = np.linspace( 0, 2*np.pi, 100)
stride_radius = 0.035 # m
circle_x = stride_radius * np.cos( theta )
circle_y = stride_radius * np.sin( theta )
circle_z = -0.03

foot_circle_center = np.array( [ [-0.16, 0.21],
                                 [0.16, 0.21],
                                 [-0.27, 0.0],
                                 [0.27, 0.0],
                                 [-0.16, -0.21],
                                 [0.16, -0.21]] ).T

for leg in range(6):
    ax.plot3D( circle_x + foot_circle_center[0,leg], circle_y + foot_circle_center[1,leg], circle_z, linewidth=4 )

# Limit the figure
ax.set_xlim( xrange )
ax.set_ylim( yrange )
ax.set_zlim( zrange )

ax.legend()
print("done...")
