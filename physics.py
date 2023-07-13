g = 9.81 # m/s^2

def calculate_buoyancy(V, density_fluid):
    """Compute and returns the buoyancy of an object in a fluid.
    Args: 
        V (int or float): Volume of liquid displaced/volume of object of object is completely submerged (m^3)
        density_fluid (int or float): The density of the fluid that the object is in/displacing (kg/m^3)
    Returns: 
        Fb (float): The buoyant force applied - calulated using archimedes principal (N)
    """
    if V or density_fluid > 0:
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

    if calculate_buoyancy(V, 1000) > (mass * g):
        return True
    if calculate_buoyancy(V, 1000) < (mass * g):
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
    density_water = 1000 # kg/m^3
    pressure = density_water * g * depth
    return pressure

if __name__ == "__main__":
    # Trial
    print(calculate_buoyancy(1, 1000))
    print(will_it_float(1, 999))
    print(calculate_pressure(100))