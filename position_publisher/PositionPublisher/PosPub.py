import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def parse_arguments():
    parser = argparse.ArgumentParser(description="Задайте точку цели передвижения робота.")
    parser.add_argument('--x', type=float, help='Х целевая координата')
    parser.add_argument('--y', type=float, help='Y целевая координата')
    return parser.parse_args()

def get_user_input():
    try:
        x = float(input("Введите Х координату: "))
        y = float(input("Введите Y координату: "))
        return x, y
    except ValueError:
        print("Некорректные значения. Пожалуйста, введите float координаты.")
        return get_user_input()

def set_and_move_to_goal(navigator, x, y):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.w = 1.0

    print(f"Перемещение к цели: X={x}, Y={y}")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f'Ожидаемое время прибытия: {eta:.0f} секунд.')

        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
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

def main():
    rclpy.init()
    navigator = BasicNavigator()

    args = parse_arguments()

    navigator.waitUntilNav2Active(localizer="bt_navigator")

    while True:
        if args.x is not None and args.y is not None:
            x, y = args.x, args.y
            args.x, args.y = None, None
        else:
            x, y = get_user_input()

        set_and_move_to_goal(navigator, x, y)

        continue_prompt = input("Хотите задать другую цель? (да/нет): ")
        if continue_prompt.lower() != 'да':
            break

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
