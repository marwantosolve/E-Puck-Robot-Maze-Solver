"""
Enhanced E-puck maze solver controller with intelligent navigation
"""

from controller import Robot, Motor, DistanceSensor
import math

# Time step in milliseconds
TIME_STEP = 64

# Constants for the robot's behavior
MAX_SPEED = 6.28  # Maximum wheel speed
WALL_THRESHOLD = 80  # Reduced threshold for wall detection
CLOSE_WALL_THRESHOLD = 130  # Adjusted threshold for very close walls
FRONT_WALL_THRESHOLD = 90  # Adjusted front wall threshold
CORNER_THRESHOLD = 100  # Lowered threshold for corner detection

# Distances for determining dead ends and intersections
DEAD_END_THRESHOLD = 3  # Number of walls to consider as dead end
INTERSECTION_THRESHOLD = 1  # Maximum number of walls to consider as intersection

# For tracking visited locations
GRID_SIZE = 0.1  # Size of each grid cell in meters
LOCATION_TOLERANCE = 0.05  # Tolerance for considering same location

# For stuck detection
STUCK_DISTANCE_THRESHOLD = 0.003  # 3mm - even more sensitive stuck detection
STUCK_TIME_THRESHOLD = 20  # Detect stuck condition faster

# Maze completion detection
# Number of unique cells to visit before considering maze solved
COMPLETION_VISIT_THRESHOLD = 50
# Distance from center to consider as outer perimeter
OUTER_PERIMETER_DISTANCE = 0.5


def run_robot(robot):
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

  # Memory for tracking visited positions
  visited_positions = set()

  # Track decision points for backtracking
  decision_points = []

  # Track previous wheel velocities for odometry
  prev_left_vel = 0
  prev_right_vel = 0

  # State variables
  # Can be FOLLOW_LEFT_WALL, FOLLOW_RIGHT_WALL, ESCAPE_CORNER, CIRCLE_PERIMETER
  state = "FOLLOW_LEFT_WALL"
  stuck_counter = 0
  last_position = [0, 0]
  escape_mode_counter = 0
  rotation_direction = 1  # 1 for right, -1 for left
  maze_completed = False
  circle_direction = "CLOCKWISE"  # Direction to circle the maze after completion
  perimeter_follow_time = 0  # Time counter for outer perimeter following

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

    # Count walls for intersection/dead-end detection
    wall_count = sum([front_left, front, front_right, right, left])

    # More precise dead end detection by checking front sensors specifically
    front_blocked = front or (front_left and front_right)
    side_blocked = (left and right) or (left_front and right)
    is_dead_end = front_blocked and side_blocked

    # Check if we're at an intersection - no walls or only one wall
    is_intersection = wall_count <= INTERSECTION_THRESHOLD

    # Add current position to visited set
    visited_positions.add(grid_pos)

    # Check if maze is completed based on number of visited cells
    if not maze_completed and len(visited_positions) >= COMPLETION_VISIT_THRESHOLD:
      print("Maze completed! Switching to outer perimeter following mode.")
      maze_completed = True
      state = "CIRCLE_PERIMETER"
      # Start with right wall following for perimeter circling
      circle_direction = "CLOCKWISE"

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
        # Return to regular wall following or perimeter circling based on maze completion
        if maze_completed:
          state = "CIRCLE_PERIMETER"
        elif rotation_direction == 1:  # If we were turning right
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
    if state == "CIRCLE_PERIMETER":
      # Robot is circling the outer perimeter after maze completion
      if circle_direction == "CLOCKWISE":
        # Follow right wall to circle clockwise
        if front or front_right:
          # Wall ahead, turn left
          left_speed = -MAX_SPEED * 0.4
          right_speed = MAX_SPEED * 0.8
        elif not right and not front_right:
          # No wall on right, turn right to follow it
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED * 0.1
        elif front_right:
          # Too close to right wall, adjust
          left_speed = MAX_SPEED * 0.6
          right_speed = MAX_SPEED
        else:
          # Follow right wall
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED
      else:  # COUNTERCLOCKWISE
        # Follow left wall to circle counterclockwise
        if front or front_left:
          # Wall ahead, turn right
          left_speed = MAX_SPEED * 0.8
          right_speed = -MAX_SPEED * 0.4
        elif not left and not left_front:
          # No wall on left, turn left to follow it
          left_speed = MAX_SPEED * 0.1
          right_speed = MAX_SPEED
        elif left_front:
          # Too close to left wall, adjust
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED * 0.6
        else:
          # Follow left wall
          left_speed = MAX_SPEED
          right_speed = MAX_SPEED

      # Periodically switch direction to make robot circle in both directions
      perimeter_follow_time += TIME_STEP / 1000
      if perimeter_follow_time > 1:  # Switch direction every 60 seconds
        if circle_direction == "CLOCKWISE":
          circle_direction = "COUNTERCLOCKWISE"
        else:
          circle_direction = "CLOCKWISE"
        perimeter_follow_time = 0

    elif state == "ESCAPE_CORNER":
      # Execute an aggressive turn to escape corner or stuck situation
      if rotation_direction == 1:  # Turn right
        left_speed = MAX_SPEED * 1.1  # Slightly higher speed for faster turn
        right_speed = -MAX_SPEED * 0.8
      else:  # Turn left
        left_speed = -MAX_SPEED * 0.8
        right_speed = MAX_SPEED * 1.1  # Slightly higher speed for faster turn
    elif is_dead_end:
      # At a dead end, make a U-turn - more aggressive than before
      print("Dead end detected - executing U-turn")
      state = "ESCAPE_CORNER"  # Use the corner escape behavior for dead ends too
      escape_mode_counter = 30  # Sufficient time to complete a U-turn
      rotation_direction = 1  # Choose a default direction for U-turn
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

    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


# Create the Robot instance and run it
if __name__ == "__main__":
  robot = Robot()
  run_robot(robot)
