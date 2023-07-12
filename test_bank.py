import unittest
import bank
from bank import Bank

class TestHello(unittest.TestCase):
    bank = Bank("bank, 1")

    def test_balance(self):
        self.assertEqual(bank.balance(1), 0)

