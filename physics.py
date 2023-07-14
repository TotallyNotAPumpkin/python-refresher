import numpy as np

g = 9.81 # m/s^2
atm_pressure = 101325 # Pa
water_density = 1000 # kg/m^3
def calculate_buoyancy(V, density_fluid):
    """Compute and returns the buoyancy of an object in a fluid.
    Args: 
        V (int or float): Volume of liquid displaced/volume of object of object is completely submerged (m^3)
        density_fluid (int or float): The density of the fluid that the object is in/displacing (kg/m^3)
    Returns: 
        Fb (float): The buoyant force applied - calulated using archimedes principal (N)
    """
    if V < 0 or density_fluid < 0:
        raise ValueError("Inputs must be positive.")
    Fb = density_fluid * V * g
    return Fb

def will_it_float(V, mass):
    """Determins if an object will float or sink in water.
    Args:
        V (float): Volume of liquid displaced/volume of object of object is completely submerged (m^3)
        mass (float): Mass of the object on Earth (kg)
    Returns:
        (boolean): True if floats, False if sinks
    """
    if V < 0 or mass < 0:
        raise ValueError("Inputs must be positive.")
    if calculate_buoyancy(V, water_density) > (mass * g):
        return True
    if calculate_buoyancy(V, water_density) < (mass * g):
        return False
    else:
        pass

def calculate_pressure(depth):
    """Compute and returns the pressure exerted on an object at a specified depth. 
    Args: 
        depth (float): depth of object in water (m)
    Returns:
        pressure (float): the pressure exerted on the object by the fluid (Pa)
    """
    if depth < 0:
        raise ValueError("Input must be positive.")
    pressure = water_density * g * depth + atm_pressure
    return pressure # Pa

def calculate_acceleration(F, m):
    """Calculates and returns acceleration of an object.
    args:
        F (float): force applied to object in Newtons
        m (float): mass of object in kG
    returns:
        acc = acceleration of object with a force (F) applied and mass (m)
    """
    if F <= 0 or m <= 0:
        raise ValueError("Invalid input value(s).")
    acc = F / m
    return acc # m/s^2

def calculate_angular_acceleration(tau, I):
    """Calculates angular acceleration of an object.
    args:
        tau (float): torque applied to object in Nm
        I (float): moment of inertia of object in kg * m^2
    """
    if tau <= 0 or I <= 0:
        raise ValueError("Invalid input value(s).")
    alpha = tau / I
    return alpha 

def calculate_torque(F_magnitude, F_direction, r):
    """Calculates and returns torque applied to an object
    args:
        F_magnitude (float): magnitude of force applied to object (N)
        F_direction (float): direction of force applied to object in degrees (degrees)
        r (float): distance of axis of rotation to point where force is applied in meters
    """
    torque = r * F_magnitude * np.sin(F_direction * 180/np.pi)
    return torque

def calculate_moment_of_inertia(m, r):
    """Calculates and returns moment of inertia of a given object.
    args:
        m (float): mass of object (kg)
        r (float): distance from axis of rotation to center of mass of object (m)
    """
    momentOfInertia = m * r**2
    return momentOfInertia

def calculate_auv_acceleration(F_magnitude, F_angle, mass = 100, volume = 0.1, thruster_distance = 0.5):
    """Calculates acceleration of AUV in 2D plane.
    args:
        F_magnitude (float): magnitude of force applied by thruster (N)
        F_angle (float): angle of force applied by thruster from x-axis (radians)
        mass (float): mass of AUV - default 100 (kg)
        volume (float): volume of AUV - default 0.1 (m^3)
        thruster_distance (float): distance from center of mass of AUV to thruster - default 0.5 (m)
    """
    if F_magnitude > 100 or F_magnitude < 0:
        raise ValueError("Value is out of thruster capabilities.")
    if F_angle > (np.pi/6) or F_angle < (-np.pi/6):
        raise ValueError("Angle of thruster may not be greater than pi/6 radians in either direction")
    Ay = F_magnitude * np.sin(F_angle) / mass
    Ax = F_magnitude * np.cos(F_angle) / mass
    netAccel = np.sqrt(Ay**2 + Ax**2)
    return netAccel

def calculate_auv_angular_acceleration(F_magnitude, F_angle, inertia = 1, thruster_distance = 0.5):
    """Calculates angular acceleration of AUV in 2D plane.
    args:
        F_magnitude (float): magnitude of force applied by thruster (N)
        F_angle (float): angle of force applied by thruster from x-axis (radians)
        inertia (float): moment of inertia of AUV - default 1 (kg * m^2)
        thruster_distance (float): distance from center of mass of AUV to thruster - default 0.5 (m)
    """
    if F_magnitude > 100 or F_magnitude < 0:
        raise ValueError("Value is out of thruster capabilities.")
    if F_angle > (np.pi/6) or F_angle < (-np.pi/6):
        raise ValueError("Angle of thruster may not be greater than pi/6 radians in either direction")
    Aa = (F_magnitude * np.sin(F_angle) * thruster_distance) / inertia
    return Aa

def calculate_auv2_acceleration(T, alpha, mass = 100):
    """Calculates the acceleration of the AUV in a 2D plane.
    args:
        T (float): np.ndarray of magnitudes of forces applied by the thrusters (N)
        alpha (float): angle of thrusters (radians)
        mass (float): mass of AUV - defualt 100 (kg)
    """
    


if __name__ == "__main__":
    # Trial
    print(calculate_buoyancy(1, water_density))
    print(will_it_float(1, 999))
    print(calculate_pressure(100))