# Типы команд маршрутного задания
byte STOP = 0                           # Остановиться в текущей позиции
byte TAKEOFF = 1                        # Взлет
byte LAND = 2                           # Посадка
byte GO_TO_POINT = 3                    # Достигнуть точки
byte GO_ALONG_TRAJECTORY = 4            # Движение по траектории
byte RETURN = 5                         # Возврат в начальную позицию

int16 cmd_num                           # Порядковый номер команды
byte type                               # Тип команды
geometry_msgs/Point[] coordinates       # Набор координат