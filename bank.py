class Bank:
    def __init__(self, name, number, bal = 0):
        self.name = name
        self.number = number
        self.bal = bal

    def wd(self, accnum, amt):
        if accnum == self.number:
            if self.bal >= amt:
                self.bal -= amt
            else: 
                print(f"You cannot withdraw more than your current balance: {self.bal}")
        else:
            print("Incorrect account number.")

    def dep(self, accnum, amt):
        if accnum == self.number:
            self.bal += amt
        else:
            print("Incorrect account number.")

    def balance(self, accnum):
        if accnum == self.number:
            print(self.bal)
        else:
            print("Incorrect account number.")

if __name__ == "__main__":
    emily = Bank("Emily Wang", 1, 0)
    emily.balance(1)
    emily.dep(1, 30)
    emily.balance(1)
    emily.wd(1, 20)