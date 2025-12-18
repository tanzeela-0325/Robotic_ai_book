---
id: data-model
title: Data Model - AI-Native Physical Humanoid Robotics
sidebar_label: Data Model
---

# Data Model: AI-Native Physical Humanoid Robotics Book

## Entity Definitions

### 1. RobotState
**Description**: Represents the current state of the humanoid robot

**Fields**:
- `timestamp`: float - Unix timestamp of state measurement
- `pose`: Pose3D - Position (x, y, z) and orientation (quaternion) in world frame
- `joint_angles`: Dict[str, float] - Mapping of joint names to current angles in radians
- `joint_velocities`: Dict[str, float] - Mapping of joint names to current velocities
- `joint_efforts`: Dict[str, float] - Mapping of joint names to current efforts/torques
- `sensors`: Dict[str, Any] - Sensor readings (IMU, force/torque, cameras, etc.)
- `battery_level`: float - Battery charge percentage (0.0-1.0)
- `operational_mode`: str - Current operational mode (idle, navigating, manipulating, etc.)
- `safety_status`: SafetyStatus - Current safety state and active constraints

**Relationships**:
- Associated with one RobotConfiguration (robot morphology)
- Contains multiple SensorReadings

### 2. NavigationGoal
**Description**: Defines a target for robot navigation

**Fields**:
- `goal_id`: str - Unique identifier for the navigation goal
- `target_pose`: Pose3D - Desired final position and orientation
- `path_constraints`: PathConstraints - Kinematic and environmental constraints
- `obstacles`: List[Obstacle] - Static and dynamic obstacles in the environment
- `costmap`: CostMap - Precomputed cost map for navigation
- `timeout`: float - Maximum time allowed for goal completion (seconds)
- `recovery_behaviors`: List[str] - Behaviors to try if navigation fails

**Relationships**:
- Associated with one RobotState (current state when goal was set)
- Generates multiple Waypoints during execution

### 3. AICommand
**Description**: Represents a high-level command from AI system to robot

**Fields**:
- `command_id`: str - Unique identifier for the command
- `natural_language`: str - Original natural language input
- `parsed_intent`: ParsedIntent - Structured representation of the intent
- `execution_plan`: List[Action] - Sequence of actions to execute
- `confidence_score`: float - Confidence in intent parsing (0.0-1.0)
- `execution_status`: str - Current status (pending, executing, completed, failed)
- `timestamp`: float - Time when command was received
- `safety_validation`: SafetyValidation - Results of safety checks

**Relationships**:
- Associated with one RobotState (state when command was received)
- Contains multiple Action objects
- May reference KnowledgeBase entries for context

### 4. SimulationParameters
**Description**: Configuration parameters for robot simulation

**Fields**:
- `physics_engine`: str - Physics engine name (e.g., "ode", "bullet", "dart")
- `update_rate`: float - Simulation update rate in Hz
- `real_time_factor`: float - Target simulation speed relative to real time
- `gravity`: Vector3 - Gravity vector (default [0, 0, -9.81])
- `solver_iterations`: int - Number of solver iterations for contact resolution
- `contact_surface_params`: ContactSurfaceParams - Contact material properties
- `sensor_noise_params`: Dict[str, NoiseParams] - Noise models for each sensor type
- `environment_properties`: Dict[str, Any] - Environmental parameters

**Relationships**:
- Applied to one RobotModel for simulation
- Associated with multiple SimulationScenario objects

### 5. SafetyBoundary
**Description**: Defines safety constraints and limits for robot operation

**Fields**:
- `boundary_id`: str - Unique identifier for the safety boundary
- `type`: str - Type of boundary (joint_limits, velocity_limits, workspace, etc.)
- `limits`: Dict[str, float] - Actual limit values (min/max for each parameter)
- `enforcement_mechanism`: str - How boundary is enforced (hard stop, warning, etc.)
- `recovery_procedure`: str - Procedure to follow when boundary is approached
- `active`: bool - Whether this boundary is currently active
- `priority`: int - Priority level for boundary enforcement (0=highest)

**Relationships**:
- Associated with one RobotConfiguration
- Monitored by SafetySupervisor system

## State Transitions

### RobotState Transitions
- `idle` → `navigating`: When NavigationGoal is received
- `navigating` → `manipulating`: When reaching navigation goal and manipulation required
- `manipulating` → `idle`: When manipulation task completed
- `any_state` → `emergency_stop`: When safety boundary violated
- `emergency_stop` → `idle`: When emergency cleared by operator

### AICommand Lifecycle
- `pending` → `validating`: When command received
- `validating` → `planning`: When safety validation passes
- `planning` → `executing`: When execution plan is ready
- `executing` → `completed`: When all actions finished successfully
- `executing` → `failed`: When action fails or safety violation occurs
- `executing` → `interrupted`: When higher priority command received

## Validation Rules

### RobotState Validation
- Joint angles must be within physical limits defined in RobotConfiguration
- Pose coordinates must be within operational workspace
- Battery level must be > 0.1 for active operations
- Sensor readings must pass plausibility checks

### NavigationGoal Validation
- Target pose must be reachable given robot kinematics
- Path must not violate any active safety boundaries
- Obstacle detection must be current (within 1 second)

### AICommand Validation
- Natural language must be parsed into valid intent
- Execution plan must not violate any safety constraints
- Required resources must be available

### SimulationParameters Validation
- Update rate must be ≥ 100Hz for stable humanoid simulation
- Gravity must be within reasonable range [-15, -5] m/s²
- Solver iterations must be ≥ 50 for stable contact resolution

### SafetyBoundary Validation
- Limits must be within physical capabilities of robot
- Priority levels must be unique within system
- Recovery procedures must be executable

## Relationships and Constraints

### Robot State Hierarchy
```
RobotConfiguration (static)
    ↓ (instantiated)
RobotState (dynamic)
    ↓ (goal-driven)
NavigationGoal → Waypoints
    ↓ (AI-driven)
AICommand → Action Sequence
    ↓ (safety-monitored)
SafetyBoundary (validation/enforcement)
```

### Simulation Context
```
SimulationParameters
    ↓ (applies to)
RobotModel → RobotState (simulated)
    ↓ (generates)
SensorSimulator → SensorReadings
    ↓ (validates)
SafetyBoundary (simulation safety)
```

This data model provides the foundation for the AI-Native Physical Humanoid Robotics book's technical content, ensuring consistency across all modules and chapters.