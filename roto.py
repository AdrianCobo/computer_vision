import numpy as np

def rototrans_y_general(rotation_angle_deg, translation_xyz):
    """
    Crea una matriz de rototraslación alrededor del eje Y con traslación arbitraria.
    
    Args:
        rotation_angle_deg (float): Ángulo de rotación en grados.
        translation_xyz (tuple): Traslación (tx, ty, tz)

    Returns:
        np.ndarray: Matriz 4x4 de rototraslación
    """
    theta = np.radians(rotation_angle_deg)
    tx, ty, tz = translation_xyz

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    R = np.array([
        [ cos_theta, 0, sin_theta],
        [        0, 1,        0],
        [-sin_theta, 0, cos_theta]
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]

    return T

# Ejemplo: dos rototraslaciones
# T1 = rototrans_y_general(0, (0.075, 0.0, -0.04330))
# T2 = rototrans_y_general(-300, (0.0, 0.0, 0.0))

T1 = np.array([[-0.5,     0.,      0.866,  -0.1125],
               [0.,      1.,      0.,      0.],
               [-0.866,   0.,     -0.5,    -0.0217],
               [0,0,0,1]])
               
T2 = np.array([[0.997002,     0.0306408,     0.0710409,  0.041638],
               [-0.0302016,      0.99952,      -0.00723911,      0.000315537],
               [-0.0712272,   0.00507176,      0.997448,    0.105469],
               [0,0,0,1]])

# Aplicación secuencial: T_total = T2 @ T1
T_total = T2 @ T1

# Mostrar resultados
np.set_printoptions(precision=4, suppress=True)
print("T1:\n", T1)
print("\nT2:\n", T2)
print("\nT_total = T2 @ T1:\n", T_total)
