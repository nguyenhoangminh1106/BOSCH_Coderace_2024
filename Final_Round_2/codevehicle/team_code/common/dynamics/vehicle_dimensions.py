from carla import Vector3D


class VehicleDimensions:
    def __init__(
        self,
        extent: Vector3D = None,
        length=None,
        width=None,
        height=None
    ):
        """	
            _extent (Vector3D): (length/2, width/2, height/2)	
            _length (float)	
            _width (float)	
            _height (float)	
        """
        self._length = 0.
        self._width = 0.
        self._height = 0.
        self._extent = Vector3D()

        if extent:
            self.extent = extent
        if length:
            self.length = length
        if width:
            self.width = width
        if height:
            self.length = height

    @property
    def extent(self):
        return self._extent

    @extent.setter
    def extent(self, var):
        self._extent = var
        self._length = var.x * 2
        self._width = var.y * 2
        self._height = var.z * 2

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, var):
        self._length = var
        self._extent.x = var / 2

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, var):
        self._width = var
        self._extent.y = var / 2

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, var):
        self._height = var
        self._extent.z = var / 2

    def __repr__(self):
        return f"({self._length:.3f}, {self._width:.3f}, {self._height:.3f})"
