# Chapter 2: Nodes, Topics, Services, and Actions

## Introduction

Understanding the communication patterns in ROS 2 is fundamental to building effective humanoid robots. The three primary communication mechanisms - nodes, topics, services, and actions - each serve distinct purposes in the distributed control of robotic systems.

## Nodes: The Building Blocks

### Node Types in Humanoid Robotics

Humanoid robots require several specialized node types:

#### Sensor Nodes
These nodes collect data from various sensors mounted on the robot:
- Camera nodes for visual perception
- IMU nodes for balance and orientation
- Force/torque sensor nodes for tactile feedback
- Joint encoder nodes for position feedback

#### Perception Nodes
Processing sensor data to extract meaningful information:
- Object detection nodes for recognizing items in the environment
- SLAM (Simultaneous Localization and Mapping) nodes for navigation
- Pose estimation nodes for tracking body parts
- Face recognition nodes for social interaction

#### Planning Nodes
Generating high-level motion and task plans:
- Motion planning nodes for trajectory generation
- Task planning nodes for complex behavior sequencing
- Path planning nodes for navigation
- Behavior selection nodes for adaptive control

#### Control Nodes
Executing low-level commands to actuators:
- Joint control nodes for precise motor positioning
- Gait control nodes for walking patterns
- Balance control nodes for stability
- Safety supervisor nodes for emergency protocols

### Node Communication Patterns

Nodes in ROS 2 can communicate through three primary mechanisms:

1. **Publish/Subscribe (Topics)**: One-to-many communication
2. **Request/Response (Services)**: One-to-one synchronous communication
3. **Goal/Result (Actions)**: One-to-one asynchronous communication with feedback

## Topics: Asynchronous Communication

### Topic Fundamentals

Topics in ROS 2 are used for asynchronous communication where publishers and subscribers are decoupled in time and space. This is particularly useful for sensor data that needs to be continuously available.

### Topic Structure

Each topic in ROS 2 has:
- **Name**: A unique identifier (e.g., `/camera/image_raw`)
- **Message Type**: The data structure being transmitted
- **Quality of Service (QoS)**: Settings for reliability and durability

### Common Topics in Humanoid Robots

#### Joint State Topic
```
/joint_states
```
This topic publishes the current state of all joints:
- `name`: Array of joint names
- `position`: Array of joint positions (radians)
- `velocity`: Array of joint velocities (rad/s)
- `effort`: Array of joint efforts (torque)

#### Sensor Data Topics
```
/camera/left/image_raw
/camera/right/image_raw
/imu/data
/lidar/scan
/force_torque/left_hand
/force_torque/right_hand
```

### QoS Settings for Humanoid Applications

Different topics require different QoS settings based on their importance:

#### Reliable Communication
- **Application**: Safety-critical control commands
- **Settings**:
  - Reliability: Reliable
  - Durability: Transient Local
  - History: Keep Last (depth = 1)

#### Best Effort Communication
- **Application**: Sensor data streams
- **Settings**:
  - Reliability: Best Effort
  - Durability: Volatile
  - History: Keep Last (depth = 10)

#### Real-time Considerations
For humanoid robots requiring real-time performance:
- Use appropriate DDS implementations (Fast DDS with real-time settings)
- Configure QoS for minimal latency
- Implement proper resource management

## Services: Synchronous Request/Response

### Service Fundamentals

Services provide synchronous communication where a client sends a request and waits for a response. This is ideal for operations that must complete before proceeding.

### Service Structure

A service in ROS 2 consists of:
- **Request Message**: Data sent to the service
- **Response Message**: Data returned by the service

### Common Services in Humanoid Robotics

#### Navigation Service
```
/navigate_to_pose
```
Request: Target pose with frame ID
Response: Success indicator and navigation time

#### Robot State Service
```
/get_robot_state
```
Request: None
Response: Complete robot state information

#### Safety Service
```
/check_safety_constraints
```
Request: Proposed action with parameters
Response: Safety assessment and recommendations

### Service Implementation Best Practices

1. **Timeout Handling**: Implement timeouts for service calls to prevent deadlocks
2. **Error Propagation**: Return meaningful error codes for troubleshooting
3. **Atomic Operations**: Ensure service operations are atomic for consistency
4. **Performance Monitoring**: Track service response times for performance analysis

## Actions: Asynchronous Goal-Oriented Communication

### Action Fundamentals

Actions are designed for long-running operations that may be cancelled or monitored for progress. They consist of:
- **Goal**: Request for an action to be performed
- **Feedback**: Progress updates during execution
- **Result**: Final outcome of the action

### Action Structure

An action in ROS 2 has three components:
- **Goal Message**: Parameters for the action
- **Feedback Message**: Progress information
- **Result Message**: Final outcome

### Common Actions in Humanoid Robotics

#### Navigation Action
```
/navigate_to_pose
```
Goal: Target pose with timeout
Feedback: Current position, distance to goal
Result: Success/failure indication, navigation time

#### Manipulation Action
```
/pick_object
```
Goal: Object description and target location
Feedback: Gripper position, object detection status
Result: Success/failure, object status

#### Walking Action
```
/walk_to_position
```
Goal: Target position and walking parameters
Feedback: Current gait phase, balance status
Result: Success/failure, final position

### Action Implementation Considerations

1. **Preemption Support**: Allow cancellation of ongoing actions
2. **Progress Monitoring**: Provide meaningful feedback during execution
3. **Error Recovery**: Handle partial failures gracefully
4. **Resource Management**: Manage computational resources during long operations

## Integration Patterns for Humanoid Robots

### Multi-Layer Communication Architecture

Humanoid robots typically employ a layered communication approach:

#### Layer 1: Low-Level Control
- Direct motor control through topics
- Real-time sensor feedback
- Emergency stop protocols

#### Layer 2: Mid-Level Coordination
- Task sequencing through services
- Resource allocation
- Safety checks

#### Layer 3: High-Level Planning
- Complex behavior through actions
- Long-term planning through services
- Human interaction through services

### Communication Flow Example

Consider a simple robot movement command:

1. **User Command**: A user requests the robot to move forward
2. **Service Call**: The command is sent via a service to the motion planner
3. **Action Initiation**: The planner initiates a navigation action
4. **Feedback Loop**: Continuous feedback from sensors and actuators
5. **Result Reporting**: Final success/failure status

## Best Practices for Humanoid Communication

### Performance Optimization

1. **Efficient Data Types**: Use compact message types where possible
2. **Appropriate Frequency**: Match data rates to actual requirements
3. **Bandwidth Management**: Prioritize critical communications
4. **Buffer Management**: Avoid buffer overflows with proper queue sizes

### Safety Considerations

1. **Redundancy**: Critical communications should have redundancy
2. **Timeouts**: All communications should have appropriate timeouts
3. **Validation**: Validate all incoming data for safety
4. **Graceful Degradation**: System should continue operating with reduced functionality

### Debugging and Monitoring

1. **Logging**: Comprehensive logging of all communication
2. **Visualization**: Tools for monitoring communication patterns
3. **Statistics**: Performance metrics collection
4. **Error Handling**: Clear error reporting for troubleshooting

## Conclusion

The communication mechanisms in ROS 2 provide the foundation for building sophisticated humanoid robots. Understanding when to use nodes, topics, services, and actions is crucial for creating reliable, efficient robotic systems. Each communication type has specific strengths and appropriate use cases that must be carefully considered in the context of humanoid robotics applications.

This chapter has explored the fundamental communication patterns that enable complex humanoid robot behavior, setting the stage for deeper exploration of specific implementations in subsequent chapters.