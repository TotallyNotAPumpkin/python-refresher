import unittest
import numpy as np
import physics

class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(1, 1000), 9810.0)
        self.assertNotEqual(physics.calculate_buoyancy(1, 1001), 9810.0)
        self.assertRaises(ValueError, physics.calculate_buoyancy, -1, 1000)
        self.assertRaises(ValueError, physics.calculate_buoyancy, 1, -1000)
    
    def test_will_it_float(self):
        self.assertTrue(physics.will_it_float(1, 10))
        self.assertFalse(physics.will_it_float(1, 1001))
        self.assertRaises(ValueError, physics.will_it_float, -1, 10)
        self.assertRaises(ValueError, physics.will_it_float, 1, -10)
        self.assertIsNone(physics.will_it_float(1, 1000))

    def test_calculate_pressure(self):
        self.assertEqual(physics.calculate_pressure(0), 101325.0)
        self.assertNotEqual(physics.calculate_pressure(1), 101325.0)
        self.assertRaises(ValueError, physics.calculate_pressure, -1)
    
    def test_calculate_acceleration(self):
        self.assertEqual(physics.calculate_acceleration(100, 10), 10)
        self.assertNotEqual(physics.calculate_acceleration(100, 12), 10)
        self.assertRaises(ValueError, physics.calculate_acceleration, -100, 10)
        self.assertRaises(ValueError, physics.calculate_acceleration, 100, -10)
    
    def test_calculate_angular_acceleration(self):
        self.assertEqual(physics.calculate_angular_acceleration(10, 5), 2)
        self.assertNotEqual(physics.calculate_angular_acceleration(10, 6), 2)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, -10, 5)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, 10, -5)
    
    def test_calculate_torque(self):
        self.assertAlmostEqual(physics.calculate_torque(10, 45, 2), 14.1421356237)
        self.assertNotEqual(physics.calculate_torque(10, 45, 4), 14.1421356237)
        self.assertRaises(ValueError, physics.calculate_torque, -10, 45, 2)
        self.assertRaises(ValueError, physics.calculate_torque, 10, 45, -2)
    
    def test_calculate_moment_of_inertia(self):
        self.assertEqual(physics.calculate_moment_of_inertia(10, 2), 40)
        self.assertNotEqual(physics.calculate_moment_of_inertia(10, 3), 40)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, -10, 2)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, 10, -2)
    
    def test_calculate_auv_acceleration(self):
        self.assertAlmostEqual(physics.calculate_auv_acceleration(10, np.pi / 12, 100, 0.1, 0.5), 0.099999999999)
        # Force magnitude exceeds thruster capabilities
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 110, np.pi / 4, 100, 0.1, 0.5)
        # Negative force magnitude
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, -10, np.pi / 4, 100, 0.1, 0.5)
        # Angle of thruster exceeds the limit (pi/6 radians)
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 10, np.pi / 3, 100, 0.1, 0.5)
        # Negative mass
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 10, np.pi / 4, -100, 0.1, 0.5)
        # Negative volume
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 10, np.pi / 4, 100, -0.1, 0.5)
        # Negative thruster distance
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 10, np.pi / 4, 100, 0.1, -0.5)

    def test_calculate_auv_angular_acceleration(self):
        self.assertAlmostEqual(physics.calculate_auv_angular_acceleration(10, np.pi/4, 1, 0.5), 14.1421356237)
        self.assertNotEqual(physics.calculate_auv_angular_acceleration(10, np.pi/4, 1, 0.5), 20)
        self.assertRaises(ValueError, physics.calculate_auv_angular_acceleration, -10, np.pi/4, 1, 0.5)
    
    def test_calculate_auv2_acceleration(self):
        self.assertTrue(np.all(np.isclose(physics.calculate_auv2_acceleration(np.array([[2], [2], [0], [0]]), 0.5, 0.5, 0.5), np.array([[6.16120922], [3.36588394]]))))
        self.assertFalse(np.all(np.isclose(physics.calculate_auv2_acceleration(np.array([[2], [0], [2], [0]]), 0.5, 0.5, 0.5), np.array([[6.16120922], [3.36588394]]))))
        self.assertRaises(TypeError, physics.calculate_auv2_acceleration, [0, 2, 0, 2], 0.5, 0.5, 0.5)
        self.assertRaises(ValueError, physics.calculate_auv2_acceleration, np.ndarray([[2], [0], [2], [0]]), 0.5, 0.5, -0.5)

    def test_calculate_auv2_angular_acceleration(self):
        self.assertAlmostEqual(physics.calculate_auv2_angular_acceleration(np.array([[2], [2], [0], [0]]), 0.5, 2, 2, 100), 0.0)
        self.assertAlmostEqual(physics.calculate_auv2_angular_acceleration(np.array([[2], [1], [0], [0]]), 0.5, 2, 2, 100), 0.027140162009891514)
        self.assertNotEqual(physics.calculate_auv2_angular_acceleration(np.array([[2], [4], [0], [0]]), 0.5, 2, 2, 100), 0.027140162009891514)
        self.assertRaises(TypeError, physics.calculate_auv2_angular_acceleration, [0, 2, 0, 2], 0.5, 2, 2, 100)
    
if __name__ == "__main__":
    unittest.main()
