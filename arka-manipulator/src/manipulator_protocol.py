#!/usr/bin/env python
from arka_manipulator.msg import ManipulatorStatus
from PySide2.QtNetwork import QUdpSocket, QHostAddress
from PySide2.QtCore import QByteArray, QDataStream, QIODevice, QObject


def two_byte_int_to_byte(integer):
    integer = int(integer)
    low_byte = integer & 0xFF
    high_byte = integer >> 8
    return low_byte, high_byte


class Direction:
    def __init__(self):
        # eachone 1byte

        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.prismatic = 0
        self.joint4 = 0
        # wrist
        self.joint5 = 0
        self.gripper = 0
        self.sensor_box = 0


class AngularSpeed:
    def __init__(self):
        # eachone 2Byte
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.prismatic = 0
        self.joint4 = 0
        # wrist
        self.joint5 = 0
        self.gripper = 0
        self.sensor_box = 0


class ResetMotor:
    def __init__(self):
        # 1Byte
        # reset mainboard joints
        self.reset_main_board_motor = 0
        self.reset_sensor_board_motor = 0


class HomePosition:
    def __init__(self):
        self.home_position = 0


class EmergencyStop:
    def __init__(self):
        self.emergency_stop = 0


class SetLED:
    def __init__(self):
        self.set_led = 0


class _ManiPulatorProtocol(QObject):
    def __init__(self, controller_ip='192.168.10.10',main_board_ip='192.168.10.170', mainboard_port='3030', sensor_board_ip='192.168.10.20', sensor_board_port='3032'):
        super(_ManiPulatorProtocol, self).__init__(None)
        self.direction = Direction()
        self.angularspeed = AngularSpeed()
        self.reset_motor = ResetMotor()
        self.home_position = HomePosition()
        self.emergency_stop = EmergencyStop()
        self.set_led = SetLED()
        # TO DO init should call from class
        self.init_sockets(controller_ip)
        self.main_board_ip = QHostAddress(main_board_ip)
        self.mainboard_port = int(mainboard_port)
        self.sensor_board_ip = QHostAddress(sensor_board_ip)
        self.sensor_board_port = int(sensor_board_port)

    def set_zero(self):
        self.direction = Direction()
        self.angularspeed = AngularSpeed()

    def init_sockets(self,controller_ip):
        # self.socket_main_board = QUdpSocket(self)
        self.socket_sensor_board = QUdpSocket(self)
        self.socket_old_main_board = QUdpSocket(self)
        self.socket_sensor_board.bind(QHostAddress(controller_ip), 3037)
        self.socket_old_main_board.bind(QHostAddress(controller_ip), 3030)

    def send_data_old_main_board(self):

        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)
        # print(self.angularspeed.joint1)
        low_byte1, high_byte1 = two_byte_int_to_byte(self.angularspeed.joint1)
        low_byte2, high_byte2 = two_byte_int_to_byte(self.angularspeed.joint2)
        low_byte3, high_byte3 = two_byte_int_to_byte(self.angularspeed.joint3)
        low_byte4, high_byte4 = two_byte_int_to_byte(
            self.angularspeed.prismatic)
        low_byte5, high_byte5 = two_byte_int_to_byte(self.angularspeed.joint4)
        low_byte6, high_byte6 = two_byte_int_to_byte(self.angularspeed.joint5)
        low_byte7, high_byte7 = two_byte_int_to_byte(
            self.angularspeed.sensor_box)

        stream.writeInt8(self.direction.joint1)
        stream.writeInt8(self.direction.joint2)
        stream.writeInt8(self.direction.joint3)
        stream.writeInt8(self.direction.prismatic)
        stream.writeInt8(self.direction.joint4)
        stream.writeInt8(self.direction.joint5)

        stream.writeInt8(self.direction.sensor_box)

        stream.writeUInt8(low_byte1)
        stream.writeUInt8(high_byte1)

        stream.writeUInt8(low_byte2)
        stream.writeUInt8(high_byte2)

        stream.writeUInt8(low_byte3)
        stream.writeUInt8(high_byte3)

        stream.writeUInt8(low_byte4)
        stream.writeUInt8(high_byte4)

        stream.writeUInt8(low_byte5)
        stream.writeUInt8(high_byte5)

        stream.writeUInt8(low_byte6)
        stream.writeUInt8(high_byte6)

        stream.writeUInt8(low_byte7)
        stream.writeUInt8(high_byte7)

        stream.writeUInt8(self.reset_motor.reset_main_board_motor)
        stream.writeUInt8(self.home_position.home_position)
        stream.writeUInt8(self.emergency_stop.emergency_stop)
        self.socket_old_main_board.writeDatagram(
            datagram, self.main_board_ip, self.mainboard_port)
    def send_sensor_board_manipulator_data(self):
        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)
        low_byte4, high_byte4 = two_byte_int_to_byte(self.angularspeed.joint4)

        stream.writeUInt8(self.direction.joint4)
        stream.writeUInt8(self.direction.joint5)
        stream.writeUInt8(self.direction.gripper)
        stream.writeUInt8(self.direction.sensor_box)
        stream.writeUInt8(low_byte4)
        stream.writeUInt8(high_byte4)
        stream.writeUInt8(self.reset_motor.reset_sensor_board_motor)
        stream.writeUInt8(self.set_led.set_led)
        self.socket_sensor_board.writeDatagram(
            datagram, self.sensor_board_ip, 3032)

    def send_main_board_manipulator_data(self):
        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)
        low_byte1, high_byte1 = two_byte_int_to_byte(self.angularspeed.joint1)
        low_byte2, high_byte2 = two_byte_int_to_byte(self.angularspeed.joint2)
        low_byte3, high_byte3 = two_byte_int_to_byte(self.angularspeed.joint3)
        stream.writeUInt8(self.direction.joint1)
        stream.writeUInt8(self.direction.joint2)
        stream.writeUInt8(self.direction.joint3)
        stream.writeUInt8(self.direction.prismatic)

        stream.writeUInt8(low_byte1)
        stream.writeUInt8(high_byte1)

        stream.writeUInt8(low_byte2)
        stream.writeUInt8(high_byte2)

        stream.writeUInt8(low_byte3)
        stream.writeUInt8(high_byte3)
        stream.writeUInt8(self.reset_motor.reset_main_board_motor)
        stream.writeUInt8(self.home_position.home_position)
        stream.writeUInt8(self.emergency_stop.emergency_stop)
        self.socket_main_board.writeDatagram(
            datagram, self.main_board_ip, self.mainboard_port)
