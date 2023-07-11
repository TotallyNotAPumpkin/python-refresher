import unittest
import hello


class TestHello(unittest.TestCase):
    def test_hello(self):
        self.assertEqual(hello.hello(), "Hello, world!")

    def test_sin(self):
        self.assertEqual(hello.sin(0), 0)
        self.assertEqual(hello.sin(1), 0.8414709848078965)
        self.assertNotEqual(hello.sin(2), 0.9092974268256345)

    def test_cos(self):
        self.assertEqual(hello.cos(0), 1)
        self.assertEqual(hello.cos(1), 0.5403023058681398)
        self.assertNotEqual(hello.cos(2), 0.5403353058681398)

    def test_tan(self):
        self.assertEqual(hello.tan(0), 0)
        self.assertEqual(hello.tan(1), 1.5574077246549023)
        self.assertNotEqual(hello.tan(2), 1.2344077245549023)

    def test_cot(self):
        self.assertEqual(hello.cot(0), float("inf"))
        self.assertEqual(hello.cot(1), 0.6420926159343306)
        self.assertNotEqual(hello.cot(2), 0.6234545159343306)

# Emily's functions

    def test_add(self):
        self.assertEqual(hello.add(0, 0), 0)
        self.assertEqual(hello.add(5, 2), 7)
        self.assertNotEqual(hello.add(3, 6), 18)

    def test_sub(self):
        self.assertEqual(hello.sub(0, 0), 0)
        self.assertEqual(hello.sub(7, 3), 4)
        self.assertNotEqual(hello.sub(81, 80), 2)

    def test_mul(self):
        self.assertEqual(hello.mul(0, 32), 0)
        self.assertEqual(hello.mul(30, 2), 60)
        self.assertNotEqual(hello.mul(23, 2), 36)

    def test_div(self):
        try:
            hello.div(6, 0)
        except ValueError as v:
            self.assertEqual(str(v), "Can't divide by zero!")
        self.assertEqual(hello.div(4, 2), 2)
        self.assertEqual(hello.div(7, 1), 7)

    def test_sqrt(self):
        self.assertEqual(int(hello.sqrt(0)), 0)
        self.assertEqual(int(hello.sqrt(64)), 8)
        self.assertNotEqual(int(hello.sqrt(36)), 86)

    def test_power(self):
        self.assertEqual(hello.power(4, 2), 16)
        self.assertEqual(hello.power(3, 3), 27)
        self.assertNotEqual(hello.power(6, 2), 333)

    def test_log(self):
        self.assertEqual(hello.log(10), 2.302585092994046)
        self.assertEqual(hello.log(40), 3.6888794541139363)
        self.assertNotEqual(hello.log(4793874), 15.382843394107775)

    def test_exp(self):
        self.assertEqual(hello.exp(4), 54.598150033144236)
        self.assertEqual(hello.exp(3), 20.085536923187668)
        self.assertNotEqual(hello.exp(0), 38)


if __name__ == "__main__":
    unittest.main()
