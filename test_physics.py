import unittest
import physics


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(1, 1000), 9810.0)
        with self.assertRaises(ValueError):
            physics.calculate_buoyancy(-30, 1000)



if __name__ == "__main__":
    unittest.main()