import argparse
import numpy as np

#==============================================================================
def rotrx( theta ):
    """ Calculates 3x3 rotation matrix around X axis
    Args:
        theta (float): Angle of rotation in radians
    Returns:
        R (3x3 numpy array): Rotation matrix
    """
    cost = np.cos(theta)
    sint = np.sin(theta)
    R = np.array( [ [1.0, 0.0, 0.0],
                    [0.0, cost, -sint],
                    [0.0, sint,  cost] ] )
    return R

#==============================================================================
def rotry( theta ):
    """ Calculates 3x3 rotation matrix around Y axis
    Args:
        theta (float): Angle of rotation in radians
    Returns:
        R (3x3 numpy array): Rotation matrix
    """
    cost = np.cos(theta)
    sint = np.sin(theta)
    R = np.array( [ [cost, 0.0, sint],
                    [0.0, 1.0, 0.0],
                    [-sint, 0.0, cost] ] )
    return R

#==============================================================================
def rotrz( theta ):
    """ Calculates 3x3 rotation matrix around Z axis
    Args:
        theta (float): Angle of rotation in radians
    Returns:
        R (3x3 numpy array): Rotation matrix
    """
    cost = np.cos(theta)
    sint = np.sin(theta)
    R = np.array( [ [cost, -sint, 0.0],
                    [sint,  cost, 0.0],
                    [0.0, 0.0, 1.0] ] )
    return R

#==============================================================================
def skew( p ):
    """Calculates 3x3 skew symmetric matrix from R^3 array p
    Args:
        p (float[3]): Array of len 3 to create matrix from
    Returns:
        P (float[3,3]): 3x3 skew symmetric matrix
    """
    P = np.array( [ [    0, -p[2],  p[1]],
                    [ p[2],     0, -p[0]],
                    [-p[1],  p[0],     0] ] )
    return P

#==============================================================================
def transform_twist( R, p, twist_a ):
    """Transforms twist from frame a to frame b,
    given rotation and translation from a to b
    Args:
        R (float[3,3]): Rotation matrix from frame a to b
        p (float[3]): Translation vector from frame a to b
        twist_a (float[6]): Twist vector to transform [w_a,v_b]
    Returns:
        twist_b (float[6]): Transformed twist vector [w_b, v_b]
    """
    T_a2b = np.zeros( (6,6) )
    T_a2b[0:3,0:3] = R
    T_a2b[3:,0:3] = skew( p ) @ R
    T_a2b[3:,3:] = R
    return T_a2b @ twist_a

#==============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument( '-t', '--theta', help='Angle of rotation about Z, radians', default=0.0)

    args = parser.parse_args()

    R = rotrz( float( args.theta ) )

    print("R = " + str(R))
