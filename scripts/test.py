

class Parent:
    def __init__(self):
        self.name = "test"
        print("Parent init")

    def run(self):
        print("Running parent")


class Child(Parent):
    def __init__(self):
        super().__init__()
        self.age = 20
        self.parent_name =  self.name

    def change_name(self):
        self.parent_name = "test2"



if __name__ == '__main__':
    c = Child()
    print(c.name)
    print(c.parent_name)

    c.change_name()

    print(c.name)
    print(c.parent_name)