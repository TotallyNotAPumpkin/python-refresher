import unittest
import physics


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(1, 1000), 9810.0)




if __name__ == "__main__":
    unittest.main()