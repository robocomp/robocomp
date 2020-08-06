

class TemplateDict(dict):
    def __init__(self):
        super().__init__()

class CustomString(str):

    def __new__(cls, value, *args, **kwargs):
        # explicitly only pass value to the str constructor
        return super(CustomString, cls).__new__(cls, value)

    def __init__(self, value, priority=0, operation=0):
        self.priority = priority
        self.operation = operation


if __name__ == '__main__':
    a = CustomString("lapatata", 10, 2)
    print(a)
    print(a.priority)
    print(a.operation)
