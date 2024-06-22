from PyQt5 import QtCore, QtWidgets

class UiApp(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)

        self.central_widget = QtWidgets.QWidget(MainWindow)
        self.central_widget.setObjectName("central_widget")
        MainWindow.setCentralWidget(self.central_widget)

        self.setup_body_controls(self.central_widget)
        self.setup_head_controls(self.central_widget)
        self.setup_battery_info(self.central_widget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def setup_body_controls(self, parent):
        self.body_control_widget = QtWidgets.QWidget(parent)
        self.body_control_widget.setGeometry(QtCore.QRect(0, 350, 321, 251))
        self.body_control_widget.setObjectName("body_control_widget")

        self.body_control_layout = QtWidgets.QGridLayout(self.body_control_widget)
        self.body_control_layout.setContentsMargins(0, 0, 0, 0)
        self.body_control_layout.setObjectName("body_control_layout")

        self.label_move_body = QtWidgets.QLabel(self.body_control_widget)
        self.label_move_body.setObjectName("label_move_body")
        self.body_control_layout.addWidget(self.label_move_body, 7, 0, 1, 1)

        self.body_forward_btn = QtWidgets.QPushButton(self.body_control_widget)
        self.body_forward_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.body_forward_btn.setObjectName("body_forward_btn")
        self.body_control_layout.addWidget(self.body_forward_btn, 7, 1, 1, 1)

        self.body_left_btn = QtWidgets.QPushButton(self.body_control_widget)
        self.body_left_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.body_left_btn.setObjectName("body_left_btn")
        self.body_control_layout.addWidget(self.body_left_btn, 8, 0, 1, 1)

        self.body_backward_btn = QtWidgets.QPushButton(self.body_control_widget)
        self.body_backward_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.body_backward_btn.setObjectName("body_backward_btn")
        self.body_control_layout.addWidget(self.body_backward_btn, 8, 1, 1, 1)

        self.body_right_btn = QtWidgets.QPushButton(self.body_control_widget)
        self.body_right_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.body_right_btn.setObjectName("body_right_btn")
        self.body_control_layout.addWidget(self.body_right_btn, 8, 2, 1, 1)

        self.label_body_linear_speed = QtWidgets.QLabel(self.body_control_widget)
        self.label_body_linear_speed.setMaximumSize(QtCore.QSize(16777215, 16))
        self.label_body_linear_speed.setObjectName("label_body_linear_speed")
        self.body_control_layout.addWidget(self.label_body_linear_speed, 2, 0, 1, 3)


        self.body_linear_speed_slider = QtWidgets.QSlider(self.body_control_widget)
        self.body_linear_speed_slider.setMaximumSize(QtCore.QSize(16777215, 20))
        self.body_linear_speed_slider.setOrientation(QtCore.Qt.Horizontal)
        self.body_linear_speed_slider.setObjectName("body_linear_speed_slider")
        self.body_control_layout.addWidget(self.body_linear_speed_slider, 3, 0, 1, 3)
        self.body_linear_speed_slider.setMinimum(0)
        self.body_linear_speed_slider.setMaximum(500)
        self.body_linear_speed_slider.setValue(100)

        self.label_body_angular_speed = QtWidgets.QLabel(self.body_control_widget)
        self.label_body_angular_speed.setMaximumSize(QtCore.QSize(16777215, 16))
        self.label_body_angular_speed.setObjectName("label_body_angular_speed")
        self.body_control_layout.addWidget(self.label_body_angular_speed, 0, 0, 1, 3)

        self.body_angular_speed_slider = QtWidgets.QSlider(self.body_control_widget)
        self.body_angular_speed_slider.setMinimumSize(QtCore.QSize(0, 20))
        self.body_angular_speed_slider.setOrientation(QtCore.Qt.Horizontal)
        self.body_angular_speed_slider.setObjectName("body_angular_speed_slider")
        self.body_control_layout.addWidget(self.body_angular_speed_slider, 1, 0, 1, 3)
        self.body_angular_speed_slider.setMinimum(0)
        self.body_angular_speed_slider.setMaximum(500)
        self.body_angular_speed_slider.setValue(100)

    def setup_head_controls(self, parent):
        self.head_control_widget = QtWidgets.QWidget(parent)
        self.head_control_widget.setGeometry(QtCore.QRect(480, 390, 321, 211))
        self.head_control_widget.setObjectName("head_control_widget")

        self.head_control_layout = QtWidgets.QGridLayout(self.head_control_widget)
        self.head_control_layout.setContentsMargins(0, 0, 0, 0)
        self.head_control_layout.setObjectName("head_control_layout")

        self.label_move_head = QtWidgets.QLabel(self.head_control_widget)
        self.label_move_head.setObjectName("label_move_head")
        self.head_control_layout.addWidget(self.label_move_head, 3, 0, 1, 1)

        self.head_forward_btn = QtWidgets.QPushButton(self.head_control_widget)
        self.head_forward_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.head_forward_btn.setObjectName("head_forward_btn")
        self.head_control_layout.addWidget(self.head_forward_btn, 3, 1, 1, 1)

        self.head_left_btn = QtWidgets.QPushButton(self.head_control_widget)
        self.head_left_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.head_left_btn.setMouseTracking(True)
        self.head_left_btn.setObjectName("head_left_btn")
        self.head_control_layout.addWidget(self.head_left_btn, 4, 0, 1, 1)

        self.head_backward_btn = QtWidgets.QPushButton(self.head_control_widget)
        self.head_backward_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.head_backward_btn.setObjectName("head_backward_btn")
        self.head_control_layout.addWidget(self.head_backward_btn, 4, 1, 1, 1)

        self.head_right_btn = QtWidgets.QPushButton(self.head_control_widget)
        self.head_right_btn.setMaximumSize(QtCore.QSize(16777215, 16777208))
        self.head_right_btn.setObjectName("head_right_btn")
        self.head_control_layout.addWidget(self.head_right_btn, 4, 2, 1, 1)

        self.label_head_speed = QtWidgets.QLabel(self.head_control_widget)
        self.label_head_speed.setMaximumSize(QtCore.QSize(16777215, 16))
        self.label_head_speed.setObjectName("label_head_speed")
        self.head_control_layout.addWidget(self.label_head_speed, 0, 0, 1, 1)

        self.head_speed_slider = QtWidgets.QSlider(self.head_control_widget)
        self.head_speed_slider.setMaximumSize(QtCore.QSize(16777215, 20))
        self.head_speed_slider.setOrientation(QtCore.Qt.Horizontal)
        self.head_speed_slider.setObjectName("head_speed_slider")
        self.head_control_layout.addWidget(self.head_speed_slider, 1, 0, 1, 3)
        self.head_speed_slider.setMinimum(0)
        self.head_speed_slider.setMaximum(500)
        self.head_speed_slider.setValue(100)

    def setup_battery_info(self, parent):
        self.battery_per_label = QtWidgets.QLabel(parent)
        self.battery_per_label.setObjectName("battery_per_label")
        self.battery_per_label.move(10, 10)

        self.battery_volt_label = QtWidgets.QLabel(parent)
        self.battery_volt_label.setObjectName("battery_volt_label")
        self.battery_volt_label.move(10, 25)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_body_linear_speed.setText(_translate("MainWindow", "Linear speed: 1.0       "))
        self.body_right_btn.setText(_translate("MainWindow", "Rotate Right: D"))
        self.label_move_body.setText(_translate("MainWindow", "Move body"))
        self.body_backward_btn.setText(_translate("MainWindow", "Backward: S"))
        self.body_forward_btn.setText(_translate("MainWindow", "Forward: W"))
        self.body_left_btn.setText(_translate("MainWindow", "Rotate Left: A"))
        self.label_body_angular_speed.setText(_translate("MainWindow", "Angular speed: 1.0      "))
        self.head_left_btn.setText(_translate("MainWindow", "Left: J"))
        self.head_forward_btn.setText(_translate("MainWindow", "Up: I"))
        self.head_right_btn.setText(_translate("MainWindow", "Right: L"))
        self.head_backward_btn.setText(_translate("MainWindow", "Down: K"))
        self.label_move_head.setText(_translate("MainWindow", "Move head"))
        self.label_head_speed.setText(_translate("MainWindow", "Speed: 1.0  "))
        self.battery_per_label.setText(_translate("MainWindow", "Battery Level: N/A       "))
        self.battery_volt_label.setText(_translate("MainWindow", "Battery Voltage: N/A       "))

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = UiApp()
        self.ui.setupUi(self)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
