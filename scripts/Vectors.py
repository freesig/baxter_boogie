class V3D:

    def __init__(self, x, y, z):
        self.__x = x;
        self.__y = y;
        self.__z = z;

    def __add__(self, vector):
        x = self.__x + vector.__x;
        y = self.__y + vector.__y;
        z = self.__z + vector.__z;
        return V3D(x, y, z);

    def __sub__(self, vector):
        x = self.__x - vector.__x;
        y = self.__y - vector.__y;
        z = self.__z - vector.__z;
        return V3D(x, y, z);

    def __eq__(self, vector):
        if self.__x == vector.__x:
            if self.__y == vector.__y:
                if self.__z == vector.__z:
                    return True;
        return False;

    def display(self):
        print "(",  self.__x, ", ", self.__y, ", ", self.__z, ")";
