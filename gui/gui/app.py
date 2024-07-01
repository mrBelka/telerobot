import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from gui.ui import UiApp
#from gui.movement_publishers import BodyMovePublisher
import rclpy
from rclpy.node import Node

from telerobot_interfaces.msg import Power
from telerobot_interfaces.msg import Battery
from geometry_msgs.msg import Twist
from telerobot_interfaces.msg import Head


class Communicate(QObject):
    battery_voltage_signal = pyqtSignal(float)
    battery_percentage_signal = pyqtSignal(float)
    body_movement_signal = pyqtSignal(float, float, float) #lx ly az
    body_speed_signal = pyqtSignal(float, float)
    head_speed_signal = pyqtSignal(float)
    head_movement_signal = pyqtSignal(int, int)


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = UiApp()
        self.ui.setupUi(self)

        self.installEventFilter(self)
        #коммуникаторы
        self.communicator = Communicate()
        self.communicator.battery_percentage_signal.connect(self.update_battery_percentage)
        self.communicator.battery_voltage_signal.connect(self.update_battery_voltage)

        #body
        self.setUpBodyButtons()
        #head
        self.setUpHeadButtons()

        #sliders
        self.ui.body_linear_speed_slider.valueChanged.connect(
            lambda: self.update_body_speed_value(self.ui.body_linear_speed_slider.value(),
                                                 self.ui.body_angular_speed_slider.value()))
        self.ui.body_angular_speed_slider.valueChanged.connect(
            lambda: self.update_body_speed_value(self.ui.body_linear_speed_slider.value(),
                                                 self.ui.body_angular_speed_slider.value()))
        self.ui.head_speed_slider.valueChanged.connect(
            lambda: self.update_head_speed_value(self.ui.head_speed_slider.value()))


    def eventFilter(self, obj, event):   ####### add head
        if event.type() == QtCore.QEvent.KeyPress:
            key = event.key()
            if key == QtCore.Qt.Key_W:   self.ui.body_forward_btn.pressed.emit()
            elif key == QtCore.Qt.Key_S: self.ui.body_backward_btn.pressed.emit()
            elif key == QtCore.Qt.Key_A: self.ui.body_left_btn.pressed.emit()
            elif key == QtCore.Qt.Key_D: self.ui.body_right_btn.pressed.emit()
            elif key == QtCore.Qt.Key_I: self.ui.head_forward_btn.pressed.emit()
            elif key == QtCore.Qt.Key_K: self.ui.head_backward_btn.pressed.emit()
            elif key == QtCore.Qt.Key_J: self.ui.head_left_btn.pressed.emit()
            elif key == QtCore.Qt.Key_L: self.ui.head_right_btn.pressed.emit()
        elif event.type() == QtCore.QEvent.KeyRelease:
            key = event.key()
            if key in (QtCore.Qt.Key_W, QtCore.Qt.Key_S, QtCore.Qt.Key_A, QtCore.Qt.Key_D):
                self.ui.body_forward_btn.released.emit()
            elif key in (QtCore.Qt.Key_I, QtCore.Qt.Key_J, QtCore.Qt.Key_K, QtCore.Qt.Key_L):
                self.ui.head_forward_btn.released.emit()

        return super().eventFilter(obj, event)

    def setUpBodyButtons(self):
        self.ui.body_forward_btn.pressed.connect(lambda: self.move_body(l_x=1.0))
        self.ui.body_forward_btn.released.connect(lambda: self.move_body())
        self.ui.body_backward_btn.pressed.connect(lambda: self.move_body(l_x=-1.0))
        self.ui.body_backward_btn.released.connect(lambda: self.move_body())
        self.ui.body_left_btn.pressed.connect(lambda: self.move_body(l_y=1.0))
        self.ui.body_left_btn.released.connect(lambda: self.move_body())
        self.ui.body_right_btn.pressed.connect(lambda: self.move_body(l_y=-1.0))
        self.ui.body_right_btn.released.connect(lambda: self.move_body())
        self.ui.body_rotate_right_btn.pressed.connect(lambda: self.move_body(a_z=1.0))
        self.ui.body_rotate_right_btn.released.connect(lambda: self.move_body())
        self.ui.body_rotate_left_btn.pressed.connect(lambda: self.move_body(a_z=-1.0))
        self.ui.body_rotate_left_btn.released.connect(lambda: self.move_body())

    def setUpHeadButtons(self):
        self.ui.head_left_btn.pressed.connect( lambda: self.move_head(-1, 0))  # такие ли значения относительно ориентированности системы координат
        self.ui.head_right_btn.pressed.connect(lambda: self.move_head(1, 0))
        self.ui.head_forward_btn.pressed.connect(lambda: self.move_head(0, 1))
        self.ui.head_backward_btn.pressed.connect(lambda: self.move_head(0, -1))

        self.ui.head_left_btn.released.connect(lambda: self.move_head(0, 0))
        self.ui.head_right_btn.released.connect(lambda: self.move_head(0, 0))
        self.ui.head_forward_btn.released.connect(lambda: self.move_head(0, 0))
        self.ui.head_backward_btn.released.connect(lambda: self.move_head(0, 0))

    def update_battery_voltage(self, battery_voltage):
        self.ui.battery_volt_label.setText(f'Battery Voltage: {battery_voltage:.2f}V')
    def update_battery_percentage(self, battery_percantage):
        self.ui.battery_per_label.setText(f'Battery Level: {battery_percantage:.2f}%')


    def move_body(self, l_x=0.0, l_y=0.0, a_z=0.0):
        self.communicator.body_movement_signal.emit(l_x, l_y, a_z)

    def move_head(self, rotate=0, pitch=0):
        self.communicator.head_movement_signal.emit(rotate, pitch)
    def update_body_speed_value(self, linear, angular):
        self.ui.label_body_linear_speed.setText(f'Linear speed: {linear/100:.2f}')
        self.ui.label_body_angular_speed.setText(f'Angular speed: {angular/100:.2f}')
        self.communicator.body_speed_signal.emit(linear/100, angular/100)

    def update_head_speed_value(self, speed):
        self.ui.label_head_speed.setText(f'Speed: {speed/100:.2f}')
        self.communicator.head_speed_signal.emit(speed/100)

class DataCollector(Node):
    def __init__(self, communicator):
        super().__init__('Data_Collector')
        self.get_logger().info("DataCollector is working")
        timer_period = 0.5


        self.batteryPer_sub_ = self.create_subscription(Battery, '/battery', self.battery_per_callback, 10)
        self.batteryVolt_sub_ = self.create_subscription(Power, '/power', self.battery_volt_callback, 10)

        self.communicator = communicator
        self.communicator.body_movement_signal.connect(self.change_movement_components)
        self.communicator.body_speed_signal.connect(self.change_body_move_speed)
        self.communicator.head_speed_signal.connect(self.change_head_move_speed)
        self.communicator.head_movement_signal.connect(self.change_head_movement_components)

        ## для body паблишера
        self.body_move_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_body_move_pub = self.create_timer(timer_period, self.publish_body_move_message)

        self.l_x = 0.0
        self.l_y = 0.0
        self.a_z = 0.0
        self.angular_speed = 1.0
        self.linear_speed = 1.0

        ## для head паблишера
        self.head_move_pub_ = self.create_publisher(Head, 'servo_commands', 10)
        self.timer_head_move_pub = self.create_timer(timer_period, self.publish_head_move_message)

        self.head_speed = 1.0
        self.rotate = 0.0
        self.pitch = 0.0

    def battery_per_callback(self, msg):
        battery_level = msg.i_load
        self.communicator.battery_percentage_signal.emit(battery_level)
    def battery_volt_callback(self, msg):
        battery_voltage = msg.voltage
        self.communicator.battery_voltage_signal.emit(battery_voltage)
    def publish_body_move_message(self):
        msg = Twist()
        msg.linear.x = self.l_x * self.linear_speed
        msg.linear.y = self.l_y * self.linear_speed
        msg.angular.z = self.a_z * self.angular_speed
        self.body_move_pub_.publish(msg)

    def change_body_move_speed(self, linear, angular):
        self.angular_speed = angular
        self.linear_speed = linear

    def change_head_move_speed(self, speed):
        self.head_speed = speed

    def change_movement_components(self, l_x=0.0, l_y=0.0, a_z=0.0):
        self.l_x = l_x
        self.l_y = l_y
        self.a_z = a_z

    def publish_head_move_message(self):
        msg = Head()
        msg.motor_pitch = round(self.pitch * self.head_speed)                ###как округлять
        msg.motor_rotate = round(self.rotate * self.head_speed)
        self.head_move_pub_.publish(msg)

    def change_head_movement_components(self, rotate, pitch):
        self.rotate = rotate
        self.pitch = pitch
def ros_spin_loop(Node):
    rclpy.spin(Node)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    mainNode = DataCollector(main_window.communicator)
    ros_thread = threading.Thread(target=ros_spin_loop, args=(mainNode,))
    ros_thread.start()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()