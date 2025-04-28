"""
Enhanced E-puck maze solver controller with intelligent navigation
This controller implements a wall-following algorithm with advanced features:
- Wall detection and avoidance
- Corner detection and escape
- Intersection handling
"""

from controller import Robot, Motor, DistanceSensor
import math

# Time step represents the basic control cycle duration in milliseconds
# 64ms provides good balance between responsiveness and processing time
TIME_STEP = 64

# Robot behavior constants
# Maximum wheel speed (rad/s) - Matches E-puck's physical limits
MAX_SPEED = 6.28
TURN_SPEED = 5.0    # Reduced speed for turns (80% of max) for better control
NORMAL_SPEED = 5.5  # Standard forward speed (87% of max) for stable movement

# Sensor threshold values (0-1000 range for proximity sensors)
# Distance to detect normal walls (lower = earlier detection)
WALL_THRESHOLD = 80
# Distance for very close walls (higher = more cautious)
CLOSE_WALL_THRESHOLD = 130
# Front wall detection (specific for forward collision avoidance)
FRONT_WALL_THRESHOLD = 90
# Corner detection sensitivity (balance between early/late detection)
CORNER_THRESHOLD = 100

# Navigation parameters
# Max walls to consider as intersection (1 = open paths available)
INTERSECTION_THRESHOLD = 1

# Position tracking parameters
GRID_SIZE = 0.1           # Size of each virtual grid cell (10cm squares)
LOCATION_TOLERANCE = 0.05  # Tolerance for position matching (5cm)

# Stuck detection parameters
STUCK_DISTANCE_THRESHOLD = 0.003  # Minimum movement expected (3mm)
STUCK_TIME_THRESHOLD = 20        # Time steps before considering robot stuck


def run_robot(robot):
  """
  Main robot control function implementing the maze solving algorithm.
  Uses a combination of wall following and intelligent navigation strategies.

  Args:
      robot: Webots Robot object representing the E-puck robot
  """
  # Initialize devices
  left_motor = robot.getDevice("left wheel motor")
  right_motor = robot.getDevice("right wheel motor")

  # Set motor positions to infinity to enable velocity control
  left_motor.setPosition(float('inf'))
  right_motor.setPosition(float('inf'))

  # Set initial motor speeds to 0
  left_motor.setVelocity(0.0)
  right_motor.setVelocity(0.0)

  # Initialize distance sensors (proximity sensors)
  ps = []
  for i in range(8):
    sensor_name = 'ps' + str(i)
    ps.append(robot.getDevice(sensor_name))
    ps[i].enable(TIME_STEP)

  # Get GPS if available (for simulation)
  gps = None
  try:
    gps = robot.getDevice("gps")
    gps.enable(TIME_STEP)
  except:
    print("GPS not available - using odometry for position estimation")

  # Initialize position tracking - use odometry if GPS not available
  position = [0, 0]  # [x, y]
  orientation = 0  # Facing direction in radians (0 = positive x-axis)
  wheel_distance = 0.052  # Distance between wheels in meters (for E-Puck)
  wheel_radius = 0.021  # Radius of wheels in meters (for E-Puck)

  # Track decision points for backtracking
  decision_points = []

  # Track previous wheel velocities for odometry
  prev_left_vel = 0
  prev_right_vel = 0

  # State machine variables
  state = "FOLLOW_LEFT_WALL"  # Initial strategy: follow left wall
  stuck_counter = 0           # Tracks consecutive steps with minimal movement
  last_position = [0, 0]      # For stuck detection
  escape_mode_counter = 0      # Duration of corner escape maneuvers
  rotation_direction = 1       # 1 = right turn, -1 = left turn

  # Main control loop
  while robot.step(TIME_STEP) != -1:
    # Get current time in seconds
    current_time = robot.getTime()

    # Get current position
    if gps:
      # If GPS is available, use it
      gps_values = gps.getValues()
      position = [gps_values[0], gps_values[2]]  # x and z
    else:
      # Otherwise use odometry to estimate position
      left_vel = left_motor.getVelocity()
      right_vel = right_motor.getVelocity()

      # Calculate forward and rotational velocities
      v = wheel_radius * (left_vel + right_vel) / 2.0
      omega = wheel_radius * (right_vel - left_vel) / wheel_distance

      # Update position and orientation
      dt = TIME_STEP / 1000.0  # Convert to seconds
      orientation += omega * dt

      # Normalize orientation to [0, 2π]
      orientation = orientation % (2 * math.pi)

      # Update position
      position[0] += v * dt * math.cos(orientation)
      position[1] += v * dt * math.sin(orientation)

      # Update previous velocities
      prev_left_vel = left_vel
      prev_right_vel = right_vel

    # Discretize position to grid
    grid_x = round(position[0] / GRID_SIZE)
    grid_y = round(position[1] / GRID_SIZE)
    grid_pos = (grid_x, grid_y)

    # Read sensor values
    ps_values = [ps[i].getValue() for i in range(8)]

    # Detect walls in each direction
    front_left = ps_values[7] > WALL_THRESHOLD
    front = ps_values[0] > WALL_THRESHOLD
    front_right = ps_values[1] > WALL_THRESHOLD
    right = ps_values[2] > WALL_THRESHOLD
    right_back = ps_values[3] > FRONT_WALL_THRESHOLD
    back = max(ps_values[3], ps_values[4]) > FRONT_WALL_THRESHOLD
    left_back = ps_values[4] > FRONT_WALL_THRESHOLD
    left = ps_values[5] > WALL_THRESHOLD
    left_front = ps_values[6] > WALL_THRESHOLD

    # Check for corner situations - detect if both front and side sensors are triggered
    in_left_corner = (front or front_left) and (
        left or left_front) and ps_values[0] > CORNER_THRESHOLD and ps_values[6] > CORNER_THRESHOLD
    in_right_corner = (front or front_right) and (
        right or front_right) and ps_values[0] > CORNER_THRESHOLD and ps_values[1] > CORNER_THRESHOLD

    # Additional corner checks for very tight corners
    tight_left_corner = ps_values[7] > CORNER_THRESHOLD * \
        1.3 and ps_values[6] > CORNER_THRESHOLD * 1.3
    tight_right_corner = ps_values[0] > CORNER_THRESHOLD * \
        1.3 and ps_values[1] > CORNER_THRESHOLD * 1.3

    # Count walls for intersection detection
    wall_count = sum([front_left, front, front_right, right, left])

    # More precise dead end detection by checking front sensors specifically
    front_blocked = front or (front_left and front_right)
    side_blocked = (left and right) or (left_front and right)

    # Check if we're at an intersection - no walls or only one wall
    is_intersection = wall_count <= INTERSECTION_THRESHOLD

    # Check if we're stuck (not moving for some time)
    distance_moved = math.sqrt((position[0] - last_position[0])**2 +
                               (position[1] - last_position[1])**2)

    if distance_moved < STUCK_DISTANCE_THRESHOLD:  # More sensitive stuck detection
      stuck_counter += 1
    else:
      stuck_counter = 0
      last_position = position.copy()

    # If in ESCAPE_CORNER mode, decrement counter
    if state == "ESCAPE_CORNER":
      escape_mode_counter -= 1
      if escape_mode_counter <= 0:
        # Return to regular wall following
        if rotation_direction == 1:  # If we were turning right
          state = "FOLLOW_LEFT_WALL"
        else:
          state = "FOLLOW_RIGHT_WALL"

    # Corner detection or stuck detection
    if (in_left_corner or in_right_corner or tight_left_corner or tight_right_corner) and state != "ESCAPE_CORNER":
      # Enter corner escape mode
      state = "ESCAPE_CORNER"
      escape_mode_counter = 25  # Increased escape time for corners

      # Choose rotation direction based on corner type
      if in_left_corner or tight_left_corner:
        rotation_direction = 1  # Turn right to escape left corner
      else:
        rotation_direction = -1  # Turn left to escape right corner
    elif stuck_counter > STUCK_TIME_THRESHOLD and state != "ESCAPE_CORNER":
      # Stuck but not detected as corner - try more aggressive escape
      state = "ESCAPE_CORNER"
      escape_mode_counter = 40  # Even longer escape for stuck situations

      # Alternate rotation direction each time we get stuck
      rotation_direction = -rotation_direction

      # Reset stuck counter
      stuck_counter = 0

    # Initialize default speeds
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # INTELLIGENT DECISION MAKING
    if state == "ESCAPE_CORNER":
      # Execute an aggressive turn to escape corner or stuck situation
      if rotation_direction == 1:  # Turn right
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED * 0.8
      else:  # Turn left
        left_speed = -TURN_SPEED * 0.8
        right_speed = TURN_SPEED
    elif is_intersection and grid_pos not in decision_points:
      # At a new intersection, remember it for possible future backtracking
      decision_points.append(grid_pos)

      # Make a decision based on current strategy
      if state == "FOLLOW_LEFT_WALL":
        if not left:
          # No wall on left, turn left
          left_speed = MAX_SPEED * 0.2
          right_speed = MAX_SPEED
        elif not front:
          # No wall in front, go straight
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED
        elif not right:
          # No wall on right, turn right
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED * 0.2
        else:
          # Surrounded by walls, turn around more aggressively
          left_speed = -MAX_SPEED * 0.7
          right_speed = MAX_SPEED * 0.7
      else:  # FOLLOW_RIGHT_WALL
        if not right:
          # No wall on right, turn right
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED * 0.2
        elif not front:
          # No wall in front, go straight
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED
        elif not left:
          # No wall on left, turn left
          left_speed = MAX_SPEED * 0.2
          right_speed = MAX_SPEED
        else:
          # Surrounded by walls, turn around more aggressively
          left_speed = MAX_SPEED * 0.7
          right_speed = -MAX_SPEED * 0.7
    else:
      # Normal wall following behavior
      if state == "FOLLOW_LEFT_WALL":
        if front or front_left:
          # Wall directly ahead or at front-left, make sharper right turn
          left_speed = MAX_SPEED * 0.9
          right_speed = -MAX_SPEED * 0.6
        elif not left and not left_front:
          # No wall on left, turn left to follow it
          left_speed = MAX_SPEED * 0.05  # More aggressive left turn
          right_speed = MAX_SPEED
        elif left_front:
          # Too close to left wall, adjust
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED * 0.5  # More aggressive adjustment
        else:
          # Follow left wall
          left_speed = MAX_SPEED * 0.9  # Slightly reduced speed for better control
          right_speed = MAX_SPEED * 0.9
      else:  # FOLLOW_RIGHT_WALL
        if front or front_right:
          # Wall directly ahead or at front-right, make sharper left turn
          left_speed = -MAX_SPEED * 0.6
          right_speed = MAX_SPEED * 0.9
        elif not right and not front_right:
          # No wall on right, turn right to follow it
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED * 0.05  # More aggressive right turn
        elif front_right:
          # Too close to right wall, adjust
          left_speed = MAX_SPEED * 0.5  # More aggressive adjustment
          right_speed = MAX_SPEED
        else:
          # Follow right wall
          left_speed = MAX_SPEED * 0.9  # Slightly reduced speed for better control
          right_speed = MAX_SPEED * 0.9

    # Ensure all motor speeds are capped at MAX_SPEED
    left_speed = min(max(-MAX_SPEED, left_speed), MAX_SPEED)
    right_speed = min(max(-MAX_SPEED, right_speed), MAX_SPEED)

    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


# Create the Robot instance and run it
if __name__ == "__main__":
  robot = Robot()
  run_robot(robot)
