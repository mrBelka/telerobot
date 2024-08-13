import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from getPoint import movementPrompt, eulerToQuaternion
import time

def parseArguments():
    parser = argparse.ArgumentParser(description="Задайте файл с точками для передвижения робота.")
    parser.add_argument('--file', type=str, default='movementPrompt', help='Путь к файлу с точками')
    return parser.parse_args()

def pauseRobot(seconds):
    print(f"Ожидание {seconds} секунд...")
    time.sleep(seconds)

def setAndMoveToGoal(navigator, x, y, offsetYawDegrees=0.0):

    qx, qy, qz, qw = eulerToQuaternion(offsetYawDegrees)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    print(f"Перемещение к цели: X={x}, Y={y}")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            #eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            #print(f'Ожидаемое время прибытия: {eta} секунд.')

            if feedback.navigation_time and Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                print("Превышено время ожидания. Отмена задачи.")
                navigator.cancelTask()
                return

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Цель достигнута!')
    elif result == TaskResult.CANCELED:
        print('Цель была отменена!')
    elif result == TaskResult.FAILED:
        print('Цель не достигнута!')
    else:
        print('Цель имеет недопустимый статус!')

class PositionPublisher(Node):
    def __init__(self, navigator):
        super().__init__('positionPublisher')
        self.navigator = navigator
        self.timer = self.create_timer(1.0, self.publishCurrentPosition)

    def publishCurrentPosition(self):
        current_pose = self.navigator.getCurrentPose()
        if current_pose:
            print(f"Текущая позиция робота: X={current_pose.pose.position.x}, Y={current_pose.pose.position.y}")

def main():
    rclpy.init()
    navigator = BasicNavigator()

    args = parseArguments()
    prompt = movementPrompt(args.file)

    navigator.waitUntilNav2Active(localizer="bt_navigator")

    position_publisher = PositionPublisher(navigator)

    for point_name, point in prompt.points.items():
        print(f"Перемещение к точке: {point_name}")
        setAndMoveToGoal(navigator, point.X, point.Y)

        # опц спросить пользователя, двигаться ли к следующей точке
        #continue_prompt = input("Перейти к следующей точке? (да/нет): ")
        #if continue_prompt.lower() != 'да':
        #    break

        #опц задержать робот на месте на n секунд
        pauseRobot(10)

    navigator.lifecycleShutdown()
    position_publisher.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
