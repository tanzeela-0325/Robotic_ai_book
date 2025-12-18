# Chapter 3: Python AI Agents with rclpy

## Introduction

Python AI agents form the cognitive layer of modern humanoid robots, bridging the gap between high-level decision-making and low-level robotic control. The `rclpy` library provides the foundation for creating these agents within the ROS 2 ecosystem.

## Understanding rclpy

### What is rclpy?

`rclpy` is the Python client library for ROS 2, providing the necessary tools to create nodes, handle communication, and interact with the ROS 2 infrastructure. It offers:

- Node creation and lifecycle management
- Publisher and subscriber functionality
- Service and action server/client implementation
- Timer and callback mechanisms
- Parameter handling and configuration

### Key Concepts in rclpy

#### Node Lifecycle
Every rclpy node follows a defined lifecycle:
1. **Initialization**: Node creation and parameter setup
2. **Activation**: Node becomes active and starts processing
3. **Deactivation**: Node temporarily stops processing
4. **Cleanup**: Resource release and shutdown

#### Callbacks and Events
rclpy uses callbacks for asynchronous event handling:
- **Timer callbacks**: Scheduled periodic execution
- **Subscription callbacks**: Triggered on message reception
- **Service callbacks**: Invoked on service request
- **Action callbacks**: Handle goal acceptance and feedback

## Creating Basic AI Agents

### Simple Node Template

```python
import rclpy
from rclpy.node import Node

class SimpleAgent(Node):
    def __init__(self):
        super().__init__('simple_agent')

        # Create a timer for periodic execution
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Create a publisher
        self.publisher = self.create_publisher(
            String, '/simple_topic', 10)

        # Create a subscriber
        self.subscription = self.create_subscription(
            String, '/simple_topic', self.subscription_callback, 10)

    def timer_callback(self):
        # Periodic task execution
        msg = String()
        msg.data = 'Hello from SimpleAgent'
        self.publisher.publish(msg)

    def subscription_callback(self, msg):
        # Handle incoming messages
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### AI Agent Architecture Patterns

#### State Machine Approach
For complex decision-making, AI agents often use finite state machines:

```python
class StateMachineAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')
        self.state = 'IDLE'

    def process_command(self, command):
        if self.state == 'IDLE':
            if command == 'start':
                self.state = 'ACTIVE'
                return 'Transitioning to active state'
        elif self.state == 'ACTIVE':
            if command == 'pause':
                self.state = 'PAUSED'
                return 'Transitioning to paused state'
            elif command == 'stop':
                self.state = 'IDLE'
                return 'Transitioning to idle state'
        return 'No change in state'
```

#### Behavior-Based Architecture
More sophisticated agents use behavior-based approaches:

```python
class BehaviorBasedAgent(Node):
    def __init__(self):
        super().__init__('behavior_based_agent')
        self.behaviors = []

    def add_behavior(self, behavior):
        self.behaviors.append(behavior)

    def evaluate_behaviors(self):
        # Evaluate all behaviors and select highest priority
        selected_behavior = max(self.behaviors,
                               key=lambda b: b.priority)
        return selected_behavior.execute()
```

## AI Agent Communication Patterns

### Sensor Data Processing

AI agents often process sensor data to make decisions:

```python
class SensorProcessingAgent(Node):
    def __init__(self):
        super().__init__('sensor_processing_agent')

        # Subscribe to multiple sensor topics
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)

    def camera_callback(self, msg):
        # Process image data
        processed_data = self.process_image(msg)
        # Publish processed data
        self.image_processed_pub.publish(processed_data)

    def imu_callback(self, msg):
        # Process IMU data for balance control
        balance_data = self.process_imu(msg)
        # Send to balance controller
        self.balance_pub.publish(balance_data)
```

### Decision Making with ROS Services

AI agents often interact with planning services:

```python
class DecisionMakingAgent(Node):
    def __init__(self):
        super().__init__('decision_making_agent')

        # Create service client for path planning
        self.path_planning_client = self.create_client(
            NavigateToPose, '/navigate_to_pose')

        # Create service server for agent commands
        self.command_service = self.create_service(
            Trigger, '/agent_command', self.command_callback)

    def make_decision(self, situation):
        # Complex decision making logic
        if situation == 'obstacle_detected':
            # Request alternative path
            future = self.path_planning_client.call_async(
                self.create_navigation_goal())
            return future
        elif situation == 'task_complete':
            # Signal completion
            return self.signal_completion()
```

## Advanced AI Agent Features

### Multi-threading Considerations

For computationally intensive tasks, AI agents may need to use threading:

```python
import threading
from concurrent.futures import ThreadPoolExecutor

class ConcurrentAgent(Node):
    def __init__(self):
        super().__init__('concurrent_agent')
        self.executor = ThreadPoolExecutor(max_workers=4)

    def process_heavy_task(self, data):
        # Heavy computation that shouldn't block the main thread
        result = self.heavy_computation(data)
        return result

    def callback_with_threading(self, msg):
        # Submit heavy computation to thread pool
        future = self.executor.submit(
            self.process_heavy_task, msg.data)
        # Handle result asynchronously
        future.add_done_callback(self.handle_result)
```

### Parameter Management

AI agents often need configurable parameters:

```python
class ConfigurableAgent(Node):
    def __init__(self):
        super().__init__('configurable_agent')

        # Declare parameters
        self.declare_parameter('sensitivity', 0.5)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('max_speed', 1.0)

        # Get parameters
        self.sensitivity = self.get_parameter('sensitivity').value
        self.timeout = self.get_parameter('timeout').value
        self.max_speed = self.get_parameter('max_speed').value

    def update_parameters(self):
        # Dynamic parameter updates
        new_sensitivity = self.get_parameter('sensitivity').value
        if new_sensitivity != self.sensitivity:
            self.sensitivity = new_sensitivity
            self.update_sensitivity(new_sensitivity)
```

## Integration with ROS 2 Systems

### ROS 2 Actions for Long-Running Tasks

AI agents often need to coordinate with long-running operations:

```python
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

class ActionAwareAgent(Node):
    def __init__(self):
        super().__init__('action_aware_agent')

        # Create action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

    def initiate_navigation(self, target_pose):
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        # Send goal and handle feedback
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        # Get feedback
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
```

## Safety and Error Handling

### Graceful Degradation

AI agents should handle failures gracefully:

```python
class SafeAgent(Node):
    def __init__(self):
        super().__init__('safe_agent')
        self.health_monitor = self.create_timer(1.0, self.health_check)

    def health_check(self):
        # Monitor system health
        if not self.is_system_healthy():
            self.enter_safe_mode()

    def is_system_healthy(self):
        # Check critical systems
        return self.check_sensors() and self.check_actuators()

    def enter_safe_mode(self):
        # Enter safe state when problems detected
        self.get_logger().warn('Entering safe mode due to system issues')
        # Stop all movement and wait for recovery
```

## Testing and Debugging

### Unit Testing AI Agents

```python
import unittest
from unittest.mock import Mock, patch

class TestAgent(unittest.TestCase):
    def setUp(self):
        # Setup test environment
        self.agent = SimpleAgent()

    def test_initialization(self):
        # Test agent initialization
        self.assertIsNotNone(self.agent)

    def test_callback_execution(self):
        # Test callback execution
        with patch.object(self.agent, 'publisher') as mock_pub:
            self.agent.timer_callback()
            mock_pub.publish.assert_called_once()
```

## Conclusion

Python AI agents with `rclpy` provide a powerful foundation for creating intelligent humanoid robots. By understanding the core concepts and implementing proper communication patterns, developers can create robust agents that seamlessly integrate with the broader ROS 2 ecosystem. The examples in this chapter demonstrate how to build agents that can process sensor data, make decisions, and interact with other components in a safe and efficient manner.

This chapter has laid the groundwork for understanding how AI agents function within the ROS 2 framework, preparing readers for more complex implementations in subsequent chapters.