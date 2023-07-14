import unittest
import physics


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(1, 1000), 9810.0)
    
    def test_calculate_acceleration(self):
        self.assertEqual(physics.calculate_acceleration(100, 10), 10)
        
if __name__ == "__main__":
    unittest.main()