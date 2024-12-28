import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R


class Robobike_Goes_Brrrr(Node):
    def __init__(self):
        super().__init__('retranslator')

        # Подписка на обновления картинки с камеры цвета
        self.sub_colorcam = self.create_subscription(
            Image,
            '/color/image',
            self.update_last_img,
            5
        )
        self.br = CvBridge()
        self.last_img = None

        # Подписка на обновления содомитрии
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.update_last_odom,
            5
        )
        self.last_odom = None

        # Публикатор сообщений движения cmd_vel
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            5
        )

        # Публикатор сигнала окончания прохождения испытаний
        self.pub_finish = self.create_publisher(
            String,
            'robot_finish',
            5
        )


        # Цветовая палитра в формате HLS (не HSL !!!)
        # ВАЖНО !!! В cv2, HLS с какого-то хрена имеет диапазоны зн-й
        # не (0-180, 0-100, 0-100), а, внезапно, (0-180, 0-255, 0-255) !!!
        self.SVETOFOR_RED = (0.0, 104.8, 255.0) #D10000
        self.SVETOFOR_YELLOW = (60.0, 127.5, 255.0) #FFFF00
        self.SVETOFOR_GREEN = (120.0, 94.4, 255.0) #00BD00
        self.SVETOFOR_GREEN_DARKENED = (120.0, 58.6, 247.3) #027202

        self.POVOROT_BLUE = (203.4, 94.4, 255.0) #0072BB
        self.POVOROT_BLUE_DARKENED = (203.4, 43.4, 242.2) #023256

        self.ROAD_LEFT = (60.0, 127.5, 255.0) #FFFF00
        self.ROAD_RIGHT = (0.0, 255.0, 0.0) #FFFFFF


        # Ожидаем прихода первых данных с сенсоров,
        # после чего начинаем прохождение испытаний.
        # Почему мы используем именно таймер?
        # -- Потому что очень и очень важно, чтобы
        # все функции были non-blocking, иначе данные
        # с сенсоров не смогут обновляться постоянно!
        self.current_competition = 1
        self.current_move = 1
        self.road_choice = 'right' # Фоллбэк
        self.time_start = None
        self.time_prev = None
        self.time_stop = None

        # Ровно по этой же причине мы храним данные,
        # связанные с текущим шагом и испытанием, как
        # св-ва класса, а не где-то внутри функции.
        self.prev_position = None
        self.prev_orientation = None
        self.total_lin_x = 0.0
        self.total_ang_z = 0.0
        self.correct_alignment_th = 0.68 # <-- Значение подобрано методом полуночного тыка
        self.angle_correction = 0.0


        # Итак, начинаем!
        self.check_for_ready = self.create_timer(0.1, self.start)


    def update_last_odom(self, req):
        self.last_odom = req.pose
        # print('Данные об одометрии обновлены!')

    def update_last_img(self, req):
        # - horizontal_fov = 1.51843645 (?)
        # - near = 0.1, far = 50, refresh = 30
        # - размер картинки: 480x848x3

        bgr_img = self.br.imgmsg_to_cv2(req, desired_encoding='bgr8')
        # bgr_img = cv2.normalize(bgr_img, None, 0, 255, cv2.NORM_MINMAX)
        hsl_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HLS)
        
        self.last_img = hsl_img
        # print('Данные с камеры цвета обновлены!', hsl_img.shape)
        # cv2.imshow('BGR', bgr_img)
        # cv2.imshow('HSL', hsl_img)
        # cv2.waitKey(1)


    # Определение процента наличия цвета (в пределах
    # некоторого порога схожести) внутри определённого
    # заданного региона изображения цветовой камеры
    def analyze_roi_for_color(self,
            color_hls, threshold,
            start_x, end_x,
            start_y, end_y
        ):
        if (
            (end_x < start_x) or (end_y < start_y)
            or (start_x < 0) or (start_y < 0)
            or (end_x > self.last_img.shape[1]) or (end_y > self.last_img.shape[0])
        ):
            self.get_logger().error('Неверно заданы координаты ROI для анализа!')
            return 0
        if (
            (threshold < 0.0) or (threshold > 1.0)
        ):
            self.get_logger().error('Неверно задан порог толерантности к цвету!')
            return 0

        roi = self.last_img[start_y:end_y, start_x:end_x]
        
        color_np = np.array(color_hls, dtype=np.float32)

        diff = np.abs(roi - color_np)
        diff[:,:,0] = [hue % 180 for hue in diff[:,:,0]] # Не допускаем выход Hue за 0..180
        
        distances = np.sqrt(np.sum(diff**2, axis=2)) # Попарные Евклидовы расстояния
        distances /= np.max(distances) # Для удобства, ограничим их между 0 и 1
        # print(distances, distances <= threshold)
        
        # Строим пороговую маску и находим % "прохождений" маски
        roi_mask = (distances <= threshold).astype(np.uint8)
        color_pct = np.sum(roi_mask > 0) / roi_mask.size
        
        # Няшная картинка для отладки
        debug_img = self.last_img.copy() # Версия в HLS
        # debug_img = cv2.cvtColor(self.last_img, cv2.COLOR_HLS2BGR) # Версия в BGR
        debug_img[start_y:end_y, start_x:end_x] = cv2.cvtColor(roi_mask*255, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(debug_img, (start_x-2, start_y-2), (end_x+1, end_y+1), color_hls, 2)
        cv2.putText(debug_img, str(color_pct), (start_x, start_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_hls, 2)
        cv2.imshow('ROI mask @ img in HLS', debug_img)
        cv2.waitKey(1)
        
        # print('Cовпадение цветов в области:', color_pct)
        return color_pct


    # Перевод кватернионов в угол поворота вдоль оси Z
    def quat_to_yaw(self, orientation):
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        rotation = R.from_quat([qx, qy, qz, qw])
        _, _, yaw = rotation.as_euler('xyz')
        return yaw


    # Отправление сообщений cmd_vel о движении/повороте
    # до момента достижения требуемых значений изменения
    # одометрии, с учётом расстояния до конечной цели
    def move_to_odom(self,
        goal_lin_x, goal_ang_z,
        speed_lin_x, speed_ang_z
    ):
        if (
            (speed_lin_x < 0) or (speed_ang_z < 0)
        ):
            self.get_logger().error('Неверно задана скорость lin_x или ang_z!')
            return 0

        curr_position = self.last_odom.pose.position
        curr_orientation = self.last_odom.pose.orientation

        if self.prev_position is None:
            self.prev_position = curr_position
        
        if self.prev_orientation is None:
            self.prev_orientation = curr_orientation

        # Находим изменение расстояния линейной скорости
        delta_lin_x = np.sqrt(
            (curr_position.x - self.prev_position.x) ** 2
            + (curr_position.y - self.prev_position.y) ** 2
        )

        # Аналогично находим изменение расстояния угловой скорости
        delta_ang_z = (
            np.pi + self.quat_to_yaw(curr_orientation)
            - self.quat_to_yaw(self.prev_orientation)
        ) % (2*np.pi) - np.pi

        # Обновляем суммарные расстояния и данные последнего шага
        self.total_lin_x += delta_lin_x
        self.total_ang_z += delta_ang_z
        self.prev_position = curr_position
        self.prev_orientation = curr_orientation
        # print(
        #     '------\n',
        #     'cp:', curr_position.x, curr_position.y, self.quat_to_yaw(curr_orientation), '\n',
        #     'pp:', self.prev_position.x, self.prev_position.y, self.quat_to_yaw(self.prev_orientation),
        #     '\n------\n',
        #     'dd:', delta_lin_x, delta_ang_z, '\n',
        #     'td:', self.total_lin_x, self.total_ang_z, '\n',
        #     'rd:', goal_lin_x - np.abs(self.total_lin_x), np.abs(np.pi + goal_ang_z - self.total_ang_z) % (2*np.pi) - np.pi,
        #     '\n------',
        # )

        res = Twist()

        # Едем линейно вперёд если ещё не проехали нужно расстояние
        if goal_lin_x - np.abs(self.total_lin_x) < (speed_lin_x / 4):
            res.linear.x = 0.0
        else:
            res.linear.x = speed_lin_x

        # Аналогично для угловой скорости (+ учитываем поворот влево/вправо)
        if np.abs(goal_ang_z - self.total_ang_z) < (speed_ang_z / 6):
            res.angular.z = 0.0
        else:
            if (goal_ang_z - self.total_ang_z) > 0:
                res.angular.z = speed_ang_z
            else:
                res.angular.z = -speed_ang_z

        self.pub_cmd_vel.publish(res)

        if res.linear.x == 0.0 and res.angular.z == 0.0:
            # Шаг выполнен, сбрасываем суммарные расстояния
            self.total_lin_x = 0.0
            self.total_ang_z = 0.0
            return 1
        else:
            # Шаг не выполнен, продолжаем
            return 0


    # Используется только для корректировки поворота,
    # для автономного движения нужен механизм похитрее!
    def correct_alignment(self, safety_threshold):
        # Анализируем положение отн. дороги на основе пары измерений
        # для левой и правой стороны; в идеале, они д.б. примерно равны.
        alignment_score_left = (
            self.analyze_roi_for_color(
                self.ROAD_LEFT, 0.2,
                66, 126, 440, 480 # 60x60
            ) +
            self.analyze_roi_for_color(
                self.ROAD_LEFT, 0.2,
                188, 218, 360, 390 # 30x50
            ) * 2/3
        )
        alignment_score_right = (
            self.analyze_roi_for_color(
                self.ROAD_LEFT, 0.2,
                130, 190, 440, 480 # 60x60
            ) +
            self.analyze_roi_for_color(
                self.ROAD_LEFT, 0.2,
                222, 252, 360, 390 # 30x50
            ) * 2/3
        )
        # print('ASL:', alignment_score_left, '| ASR:', alignment_score_right, '| ASD:', np.abs(alignment_score_left-alignment_score_right))

        # Отсекаем случаи, когда дороги почему-то не видно
        if ((alignment_score_left < (safety_threshold / 2))
           and (alignment_score_right < (safety_threshold / 2))):
            # Корректировка не выполнена
            return 0

        # Делаем мааааленький, постепенно (но не мгновенно) затухающий,
        # поворот чтобы робот повернул немного более параллельно дороге
        res = Twist()
        if (np.abs(alignment_score_left-alignment_score_right) > safety_threshold):
            self.angle_correction = np.tanh(alignment_score_left-alignment_score_right)
            res.angular.z = self.angle_correction
            self.pub_cmd_vel.publish(res)
            print('Установлен корректирующий поворот:', res.angular.z)
        elif (np.abs(self.angle_correction) > 0.01):
            self.angle_correction /= 3 # <-- Значение подобрано методом научного "ДА РАБОТАЙ ТЫ!!1"
            res.angular.z = self.angle_correction
            self.pub_cmd_vel.publish(res)
            print('Корректирующий поворот понижен до:', res.angular.z)
        # Корректировка установлена
        return 1


    # Добро пожаловать на лекцию по Механике в ИИР НГУ!
    def start(self):
        if self.last_img is None or self.last_odom is None:
            self.get_logger().info('Ожидаю ответа от сенсоров камеры и одометрии...')
            return
        else:
            print('Испытание', self.current_competition, '| Шаг', self.current_move)

            self.correct_alignment(self.correct_alignment_th)

            match self.current_competition:
                case 1: # === ИСПЫТАНИЕ 1/3 ===
                    match self.current_move:
                        case 1:
                            # Испольщуем линейно снижающийся порог при анализе ROI
                            self.better_safe_than_sorry = 0.36 # С учётом min 4 сек ожидания
                            self.current_move += 1
                        case 2:
                            # Ждём, пока в области зелёного цвета на светофоре
                            # наконец не появится этот самый зелёный цвет
                            if self.analyze_roi_for_color(
                                # self.SVETOFOR_GREEN, 0.3,
                                self.SVETOFOR_GREEN_DARKENED, 0.3,
                                520, 680, 240, 320
                            ) > self.better_safe_than_sorry:
                                self.current_move += 1
                            else:
                                self.better_safe_than_sorry -= 0.0005
                        case 3:
                            # С этого момента начинается отсчёт времени начала заезда!
                            self.time_start = time.time()
                            self.get_logger().warn('Ну чо народ, погнали... !')
                            self.current_move += 1
                        case 4:
                            # Едем по прямой до поворота
                            self.current_move += self.move_to_odom(0.8, 0.0, 0.3, 0.0)
                        case 5:
                            # Выполняем поворот, очень стараемся не въехать в знак
                            self.current_move += self.move_to_odom(0.24, np.pi/2, 0.1, 0.3)
                        case 6:
                            # Проезжаем кусочек дороги после поворота
                            self.current_move += self.move_to_odom(0.706, 0.0, 0.3, 0.0)
                        case _:
                            # Сообщаем прошедшее с начала отсчёта время, записываем текущее
                            self.get_logger().warn(f'Испытание 1 завершено! Времени прошло: {time.time() - self.time_start}')
                            self.time_prev = time.time()

                            # Переходим к следующему испытанию
                            self.current_move = 1
                            self.current_competition += 1
                 
                case 2: # === ИСПЫТАНИЕ 2/3 ===
                    match self.current_move:
                        case 1:
                            # Выполняем поворот
                            self.current_move += self.move_to_odom(0.2, np.pi/2-0.2, 0.2, 0.4)
                        case 2:
                            # Проезжаем закуток до развилки
                            self.current_move += self.move_to_odom(0.20, 0.0, 0.3, 0.0)
                        case 3:
                            # Испольщуем линейно снижающийся порог при анализе ROI
                            self.better_safe_than_sorry = 0.4 # С учётом min 4 сек ожидания
                            self.current_move += 1
                        case 4:
                            # Сравниваем наличие синего цвета в областях
                            # появления каждого из знаков, на основании
                            # этого делаем выбор дороги, куда ехать дальше
                            left_proba = self.analyze_roi_for_color(
                                    # self.POVOROT_BLUE, 0.4,
                                    self.POVOROT_BLUE_DARKENED, 0.4,
                                    220, 390, 70, 220
                            )
                            right_proba = self.analyze_roi_for_color(
                                    # self.POVOROT_BLUE, 0.4,
                                    self.POVOROT_BLUE_DARKENED, 0.4,
                                    420, 570, 70, 220
                            )
                            if (left_proba >= right_proba) and (left_proba > self.better_safe_than_sorry):
                                self.road_choice = 'left'
                                self.get_logger().info(f'Выбранная дорога: {self.road_choice}')
                                self.current_move += 1
                            elif (left_proba < right_proba) and (right_proba > self.better_safe_than_sorry):
                                # self.road_choice = 'right'
                                self.get_logger().info(f'Выбранная дорога: {self.road_choice}')
                                self.current_move += 1
                            else:
                                self.better_safe_than_sorry -= 0.001
                        case 5:
                            # self.road_choice = 'right'
                            # Делаем поворот до сегмента кольца
                            if (self.road_choice == 'right'):
                                self.current_move += self.move_to_odom(0.4, -np.pi/2, 0.06, 0.6)
                            else:
                                self.current_move += self.move_to_odom(0.4, +np.pi/2, 0.06, 0.6)
                        case 6:
                            # Делаем большой поворот на кольце
                            if (self.road_choice == 'right'):
                                self.current_move += self.move_to_odom(0.8, +np.pi-0.1, 0.2, 0.6)
                            else:
                                self.current_move += self.move_to_odom(0.8, -np.pi+0.1, 0.2, 0.6)
                        case 7:
                            # Делаем поворот после проезда кольца
                            if (self.road_choice == 'right'):
                                self.current_move += self.move_to_odom(0.48, -np.pi/2+0.2, 0.2, 0.6)
                                # self.current_move += self.move_to_odom(0.48, -np.pi/2, 0.24, 0.6)
                            else:
                                self.current_move += self.move_to_odom(0.68, +np.pi/2-0.2, 0.2, 0.6)
                                # self.current_move += self.move_to_odom(0.68, +np.pi/2, 0.24, 0.6)
                        case 8:
                            # Проезжаем оставшийся кусочек дороги до поворота
                            if (self.road_choice == 'right'):
                                self.current_move += self.move_to_odom(0.24, -0.4, 0.2, 0.2)
                            else:
                                self.current_move += self.move_to_odom(0.08, -0.4, 0.1, 0.2)
                        case _:
                            # Сообщаем прошедшее с прошлого испытания время, записываем текущее
                            self.get_logger().warn(f'Испытание 2 завершено! Времени прошло: {time.time() - self.time_prev}')
                            self.time_prev = time.time()

                            # Переходим к следующему испытанию
                            self.current_move = 1
                            self.current_competition += 1

                case 3: # === ИСПЫТАНИЕ 3/3 ===
                    match self.current_move:
                        case 1:
                            # Медленно и аккуратно поворачиваем и двигаемся ближе к краю дороги
                            if self.move_to_odom(0.08, -np.pi/3, 0.1, 0.2):
                                # Нам очень важно, чтобы робот макисмально скорректировал свой
                                # угол поророта относительно дороги перед началом этого испытания
                                self.correct_alignment_th = 0.54 # <-- Значение подобрано...
                                                                 # ну, я думаю, уже понятно как
                                self.current_move += self.move_to_odom(0.0, 0.0, 0.0, 0.0)
                        case 2:
                            # Едем до края
                            self.current_move += self.move_to_odom(0.85, -0.2, 0.2, 0.1)
                        case 3:
                            # Поворачиваем робота
                            self.current_move += self.move_to_odom(0.0, -np.radians(85), 0.0, 0.3)
                        case 4:
                            # Едем дальше (ёпсель-мопсель, 4 часа усрал на начало третьего испытания!!!)
                            # self.current_move += self.move_to_odom(0.744*2, 0.0, 0.3, 0.0)
                            self.current_move += self.move_to_odom(1.3, 0.0, 0.2, 0.0)
                        case 5:
                            # Поворот налево, на строительную площадку
                            self.current_move += self.move_to_odom(0.0, np.radians(40), 0.0, 0.2)
                        case 6:
                            # Объезд 1
                            # self.get_logger().info('Запускаю программу "контр-аварийная езда 1"')
                            self.current_move += self.move_to_odom(0.12, 0.0, 0.1, 0.0)
                        case 7:
                            # Снова поворот налево
                            self.current_move += self.move_to_odom(0.0, np.radians(40), 0.0, 0.2)
                        case 8:
                            # Объезд 2
                            # self.get_logger().info('Запускаю программу "аварийная контр-езда 1"')
                            self.current_move += self.move_to_odom(0.38, 0.0, 0.2, 0.0)
                        case 9:
                            # Снова налево
                            self.current_move += self.move_to_odom(0.0, np.radians(75), 0.0, 0.3)
                        case 10:
                            # Объезд 3
                            self.current_move += self.move_to_odom(0.32, 0.0, 0.2, 0.0)
                        case 11:
                            # Поворот направо
                            self.current_move += self.move_to_odom(0.0, np.radians(-75), 0.0, 0.3)
                        case 12:
                            # Объезд 4
                            self.current_move += self.move_to_odom(0.34, 0.0, 0.2, 0.0)
                        case 13:
                            # Снова поворот направо
                            self.current_move += self.move_to_odom(0.0, np.radians(-75), 0.0, 0.3)
                        case 14:
                            # Объезд 5
                            self.current_move += self.move_to_odom(0.32, 0.0, 0.2, 0.0)
                        case 15:
                            # Финальный поворот налево
                            self.current_move += self.move_to_odom(0.0, np.radians(75), 0.0, 0.3)
                        case 16:
                            # Выезжаем из туннеля
                            self.current_move += self.move_to_odom(0.26, np.pi/2+0.2, 0.4, 0.2) # Чтобы эпично влететь в знак парковки
                        case 17:
                            # Наконец, уезжаем со стройплощадки!!!11
                            self.current_move += self.move_to_odom(0.8, 0.0, 0.2, 0.0)
                        case _:
                            # Сообщаем прошедшее с прошлого испытания время, записываем текущее
                            self.get_logger().warn(f'Испытание 3 завершено! Времени прошло: {time.time() - self.time_prev}')
                            self.time_prev = time.time()

                            # Переходим к следующему испытанию
                            self.current_move = 1
                            self.current_competition += 1

                case 4: # === КОНЕЦ ЗАЕЗДА ===
                    match self.current_move:
                        case 1:
                            # На всякий случай выполняем полную остановку (согласно правилам,
                            # перед завершением заезда робот должен полностью остановиться)
                            self.current_move += self.move_to_odom(0.2, 0.0, 0.02, 0.0)
                        case 2:
                            # И уже теперь записываем время окончания заезда
                            self.time_stop = time.time()

                            # Всё! Сообщаем об окончании прохождения
                            req_finish = String()
                            req_finish.data = f'Наш робот проехал первые 3 испытания и, очень уставший, но довольный собой, заснул прямо у знака парковки... Спасибо за внимание! \n\nКоманда: ROSticks | Итоговое время: {self.time_stop - self.time_start}\n'
                            self.pub_finish.publish(req_finish)
                            self.get_logger().info(req_finish.data)

                            # Переходим к следующему испытанию
                            self.current_move = 1
                            self.current_competition += 1

                case _: # Полное прекращение работы рос2 в России
                    self.get_logger().warn('ЦЕ КІНЕЦЬ...')
                    raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    robobike = Robobike_Goes_Brrrr()

    try:
        rclpy.spin(robobike)
    except SystemExit:
        robobike.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
