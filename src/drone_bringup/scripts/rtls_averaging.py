#!/usr/bin/env python
# coding=utf8

"""
Get position to TF drom RTLS system
"""

import rospy
import requests
from requests.auth import HTTPBasicAuth
import tf
from std_msgs.msg import Float32


# init params
RTLS_URL = 'http://192.168.128.177/CFG-API/monitor/tags'
RTLS_LOGIN = 'system'
RTLS_PASSWD = 'admin'
RTLS_REQUEST_DELAY = 10
ROBOT_LABEL_FILTER = [20000984, 20000987, 20000983, 20000985]

offset = [-0.25, -7.0, 0.15]
alpha = -1.74533

# коэффициент доверия текущим показаниям rtls
K_stab = 0.3

# flags
kal_flag = True
filter_flag = True

# topics
alt_topic = "/drone/alt"

alt_msg = Float32()

def pub_alt(alt):
    """
    Публикуем данные о высоте
    :param alt:
    :return:
    """
    global alt_msg
    alt_msg.data = alt
    alt_pub.publish(alt_msg)


if __name__ == '__main__':
    try:
        # инициализируем ноду
        rospy.init_node('rtls', anonymous=True)
        # задаем частоту работы
        rate = rospy.Rate(RTLS_REQUEST_DELAY)  # 10hz
        # объявляем паблишер координат ртлс
        # rtls_pub = rospy.Publisher("/rtls_coords", PoseStamped, queue_size=10)
        alt_pub = rospy.Publisher(alt_topic, Float32, queue_size=1)

        while not rospy.is_shutdown():
            # объявляем/обнуляем сообщение
            # rtls_pose_msg = PoseStamped()
            try:
                # авторизовываемся на сервере ртлс и снимаем данные
                r = requests.get(RTLS_URL, auth=HTTPBasicAuth(RTLS_LOGIN, RTLS_PASSWD))
                # пихаем данные в json
                jsonRawData = r.json()
                # вытаскиваем данные из json'a
                rawData = [x for x in jsonRawData['items'] if
                           'le_status' in x and x['le_status'] == 'HEALTH' and x['sn'] in ROBOT_LABEL_FILTER]
                # объявляем/обнуляем суммы
                x_sum = 0.0
                y_sum = 0.0
                z_sum = 0.0
                for i in range(0, len(rawData)):
                    # суммируем координаты меток
                    x_sum += rawData[i]['x'] + offset[0]
                    y_sum += rawData[i]['y'] + offset[1]
                    z_sum += rawData[i]['z'] + offset[2]
                try:
                    # вычисляем средние кординаты
                    x_aver = x_sum / len(rawData)
                    y_aver = y_sum / len(rawData)
                    z_aver = z_sum / len(rawData)

                    # объявляем вещатель TF
                    br = tf.TransformBroadcaster()

                    # если фильтруем
                    if filter_flag:
                        # если флаг тру, для 1го прогона
                        # приравниваем текущие калмановские коорды к показаниям ртлс
                        if kal_flag:
                            x_kal = x_aver
                            y_kal = y_aver
                            z_kal = z_aver
                            kal_flag = False
                        # иначе вычисляем по формуле
                        else:
                            # упрощенный фильтр Калмана для 3х координат
                            x_kal = K_stab * x_aver + (1 - K_stab) * x_kal_prev
                            y_kal = K_stab * y_aver + (1 - K_stab) * y_kal_prev
                            z_kal = K_stab * z_aver + (1 - K_stab) * z_kal_prev
                        # записываем в предыдущие значения текущие
                        x_kal_prev = x_kal
                        y_kal_prev = y_kal
                        z_kal_prev = z_kal
                        # вещаем фильтрованный неповернутый tf
                        br.sendTransform((x_kal, y_kal, z_kal),
                                         tf.transformations.quaternion_from_euler(0, 0, 0),
                                         rospy.Time.now(),
                                         '/rtls/drone'.format('drone'),
                                         "rtls")

                        # публикуем высоту
                        pub_alt(z_kal)

                    # если не фильтруем
                    else:
                        # вещаем нефильтрованный и неповернутый tf средней точки
                        br.sendTransform((x_aver, y_aver, z_aver),
                                         tf.transformations.quaternion_from_euler(0, 0, 0),
                                         rospy.Time.now(),
                                         '/rtls/drone'.format('drone'),
                                         "rtls")
                        # публикуем высоту
                        pub_alt(z_aver)
                except:
                    pass
            except:
                print ("not rtls")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
