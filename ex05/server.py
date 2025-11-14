import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_tutorials_interfaces.action import CleaningTask

class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'cleaning_task',
            self.execute_callback
        )
        
        self.pose = None
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Cleaning Action Server started')

    def pose_callback(self, msg):
        self.pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing task: {goal_handle.request.task_type}')
        
        result = CleaningTask.Result()
        feedback = CleaningTask.Feedback()
        
        task_type = goal_handle.request.task_type
        area_size = goal_handle.request.area_size
        
        if task_type == 'clean_square':
            success, cleaned_points, total_distance = self.clean_square(goal_handle, area_size, feedback)
        elif task_type == 'clean_circle':
            success, cleaned_points, total_distance = self.clean_circle(goal_handle, area_size, feedback)
        elif task_type == 'return_home':
            success, cleaned_points, total_distance = self.return_home(
                goal_handle, 
                goal_handle.request.target_x, 
                goal_handle.request.target_y, 
                feedback
            )
        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            success = False
            cleaned_points = 0
            total_distance = 0.0

        result.success = success
        result.cleaned_points = cleaned_points
        result.total_distance = total_distance
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result

    def generate_square_points(self, start_x, start_y, side_length, step=0.2):
        """Генерирует точки для движения змейкой по квадрату"""
        points = []
        x, y = start_x, start_y
        
        # Количество рядов
        num_rows = int(side_length / step)
        
        for row in range(num_rows):
            if row % 2 == 0:
                # Четные ряды: двигаемся вправо
                points.append((start_x + side_length, y))
            else:
                # Нечетные ряды: двигаемся влево
                points.append((start_x, y))
            
            # Переходим на следующий ряд
            y += step
        
        # Добавляем последнюю точку для завершения квадрата
        if num_rows % 2 == 0:
            points.append((start_x + side_length, start_y + side_length))
        else:
            points.append((start_x, start_y + side_length))
            
        return points

    def move_to_point(self, goal_handle, target_x, target_y, feedback, linear_speed=0.5, angular_speed=1.0, tolerance=0.1):
        """Двигается к указанной точке с контролем прогресса"""
        if self.pose is None:
            return False, 0.0
            
        total_distance = 0.0
        start_x, start_y = self.pose.x, self.pose.y
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                return False, total_distance
            
            # Вычисляем расстояние и угол до цели
            dx = target_x - self.pose.x
            dy = target_y - self.pose.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            
            # Проверяем достигли ли цели
            if distance < tolerance:
                self.stop_robot()
                return True, total_distance
            
            # Вычисляем разницу углов
            angle_diff = target_angle - self.pose.theta
            # Нормализуем угол в диапазон [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Управление движением
            twist = Twist()
            
            if abs(angle_diff) > 0.1:
                # Поворачиваем к цели
                twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
            else:
                # Двигаемся вперед
                twist.linear.x = min(linear_speed, distance)
                total_distance += 0.05  # Примерное расстояние за цикл
            
            self.publisher.publish(twist)
            
            # Обновляем feedback
            progress = max(0, min(100, int((1 - distance / math.sqrt((target_x-start_x)**2 + (target_y-start_y)**2)) * 100)))
            feedback.progress_percent = progress
            feedback.current_x = self.pose.x
            feedback.current_y = self.pose.y
            goal_handle.publish_feedback(feedback)
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return False, total_distance

    def clean_square(self, goal_handle, side_length, feedback):
        if self.pose is None:
            return False, 0, 0.0
            
        start_x, start_y = self.pose.x, self.pose.y
        
        # Генерируем точки для движения
        points = self.generate_square_points(start_x, start_y, side_length)
        
        total_points = len(points)
        cleaned_points = 0
        total_distance = 0.0
        
        self.get_logger().info(f'Starting square cleaning from ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'Generated {total_points} points for cleaning')
        
        # Последовательно движемся по всем точкам
        for i, (target_x, target_y) in enumerate(points):
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                return False, cleaned_points, total_distance
            
            self.get_logger().info(f'Moving to point {i+1}/{total_points}: ({target_x:.2f}, {target_y:.2f})')
            
            # Двигаемся к точке
            success, distance = self.move_to_point(goal_handle, target_x, target_y, feedback)
            
            if not success:
                return False, cleaned_points, total_distance
            
            total_distance += distance
            cleaned_points = i + 1
            
            # Обновляем прогресс
            progress = int((cleaned_points / total_points) * 100)
            feedback.progress_percent = progress
            feedback.current_cleaned_points = cleaned_points
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(f'Reached point {i+1}/{total_points}, progress: {progress}%')
        
        self.get_logger().info(f'Square cleaning completed: {cleaned_points} points, {total_distance:.2f} meters')
        return True, total_points, total_distance
    

    def return_home(self, goal_handle, target_x, target_y, feedback):
        if self.pose is None:
            return False, 0, 0.0
            
        total_distance = 0.0
        
        self.get_logger().info(f'Returning home to ({target_x:.2f}, {target_y:.2f})')
        
        success, distance = self.move_to_point(goal_handle, target_x, target_y, feedback)
        total_distance += distance
        
        if success:
            self.get_logger().info('Successfully returned home')
            return True, 0, total_distance
        else:
            self.get_logger().error('Failed to return home')
            return False, 0, total_distance

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    cleaning_action_server = CleaningActionServer()
    rclpy.spin(cleaning_action_server)
    cleaning_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()