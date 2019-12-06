class Obstacle(object):

    def __init__(self):
        self.area = -1
        self.x = -1
        self.y = -1
        self.angle = -100
        self.width = -1
        self.height = -1
        self.position = 'None'
        

    def update(self, x, y, width, height, area, angle, pos):
        self.area = area
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = angle
        self.position = pos


class Obstacles(object):

    def __init__(self):
        self.cup = Obstacle()
        self.yellow = Obstacle()
        self.blue = Obstacle()
