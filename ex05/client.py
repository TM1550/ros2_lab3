import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import CleaningTask

class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'cleaning_task')
        self.get_logger().info('Cleaning Action Client started')

    def send_goal(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, '
                              f'cleaned_points={result.cleaned_points}, '
                              f'total_distance={result.total_distance:.2f}')
        
        # Send next goal in sequence
        if hasattr(self, '_current_goal_index'):
            self._current_goal_index += 1
            if self._current_goal_index < len(self.goal_sequence):
                self.send_next_goal()
            else:
                self.get_logger().info('All tasks completed!')
                rclpy.shutdown()
        else:
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.progress_percent}% complete, '
                              f'cleaned {feedback.current_cleaned_points} points, '
                              f'position: ({feedback.current_x:.2f}, {feedback.current_y:.2f})')

    def execute_sequence(self):
        # Define cleaning sequence - only square cleaning for now
        self.goal_sequence = [
            ('clean_square', 3.0, 0.0, 0.0),  # Clean 3x3 square using snake pattern
            ('return_home', 0.0, 5.5, 5.5)    # Return to home position
        ]
        
        self._current_goal_index = 0
        self.send_next_goal()

    def send_next_goal(self):
        task_type, area_size, target_x, target_y = self.goal_sequence[self._current_goal_index]
        self.get_logger().info(f'Sending goal: {task_type}')
        self.send_goal(task_type, area_size, target_x, target_y)

def main(args=None):
    rclpy.init(args=args)
    action_client = CleaningActionClient()
    action_client.execute_sequence()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()