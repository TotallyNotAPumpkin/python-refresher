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

if __name__ == "__main__":
    # Trial
    print(calculate_buoyancy(1, water_density))
    print(will_it_float(1, 999))
    print(calculate_pressure(100))