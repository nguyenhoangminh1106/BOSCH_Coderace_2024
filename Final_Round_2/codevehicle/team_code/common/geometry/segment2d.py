from .point2d import Point2D


class Segment2D:
    def __init__(self, p1=Point2D(), p2=Point2D()):
        self.p1 = p1
        self.p2 = p2

    def ratio_from_point(self, p: Point2D):
        return self.ratio_from_coord(p.x, p.y)

    def ratio_from_coord(self, x, y):
        dx = self.p2.x - self.p1.x
        dy = self.p2.y - self.p1.y
        det = dx**2+dy**2
        ratio = (dy*(y - self.p1.y) + dx*(x - self.p1.x))/det
        num = dx*y - dy*x - self.p2.x*self.p1.y + self.p1.x*self.p2.y
        return ratio, num, det
