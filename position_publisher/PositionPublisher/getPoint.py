from scipy.spatial.transform import Rotation as R


class point:
    def __init__(self, name, X, Y):
        self.name = name
        self.X = X
        self.Y = Y

    def __repr__(self):
        return f"{self.name}: ({self.X}; {self.Y})"

class movementPrompt:
    def __init__(self, filepath):
        self.points = {}
        self.parseFile(filepath)

    def parseFile(self, filepath):
        with open(filepath, 'r') as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if len(parts) >= 4:
                    name = parts[0]  #
                    coordinates = parts[2] + " " + parts[3]
                    coordinates = coordinates.strip('()')
                    x_str, y_str = coordinates.split(';')
                    try:
                        x = float(x_str)
                        y = float(y_str)
                        self.points[name] = point(name, x, y)
                    except ValueError:
                        print(f"Не удалось преобразовать координаты в float: {coordinates}")
                else:
                    print(f"Предупреждение! Строка не соответствует ожидаемому формату: {line.strip()}")

    def getPoint(self, name):
        return self.points.get(name)

    def __repr__(self):
        return '\n'.join(str(point) for point in self.points.values())

def eulerToQuaternion(yawDegrees):
    r = R.from_euler('z', yawDegrees, degrees=True)
    return r.as_quat()