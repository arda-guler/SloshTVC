import math

class vec2:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def dot(self, ov):
        return self.x * ov.x + self.y * ov.y

    def mag(self):
        return (self.x**2 + self.y**2)**(0.5)

    def normalized(self):
        if self.mag():
            return vec2(self.x/self.mag(), self.y/self.mag())
        else:
            return vec2()

    def __repr__(self):
        return "<" + str(self.x) + ", " + str(self.y) + ">"

    def __add__(self, other):
        return vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, s):
        return vec2(self.x * s, self.y * s)

    def __truediv__(self, s):
        return vec2(self.x / s, self.y / s)

    def rotated(self, rot):
        x = self.x
        y = self.y
        x = x * math.cos(rot) - y * math.sin(rot)
        y = x * math.sin(rot) + y * math.cos(rot)

        return vec2(x, y)
