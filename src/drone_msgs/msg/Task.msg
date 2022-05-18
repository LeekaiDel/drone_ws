byte START = 0            # Запуск задания
byte PAUSE = 1            # Приостановка выполнения задания
byte END = 2              # Завершение задания
byte BRAKE = 3            # Задание прервано

std_msgs/Header header
int16 priority          # Приоритет маршрутного задания
TaskCmd[] commands      # Список команд маршрутного задания
byte state              # Состояние маршрутного задания
