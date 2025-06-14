#!/usr/bin/env python3

"""
SO-100 Follower Robot Controller with Auto-Calibration
======================================================

This class provides:
1. Auto-calibration by detecting torque limits for each joint
2. Waypoint navigation with configurable speed and acceleration
3. Safe movement within calibrated ranges
4. Automatic calibration file management

Usage:
    robot = Robot()
    robot.connect()
    
    # Auto-calibrate if needed
    if not robot.has_calibration():
        robot.auto_calibrate()
    
    # Move through waypoints
    waypoints = [
        [2000, 1500, 2000, 1800, 2000, 2040],  # Joint positions
        [2200, 1600, 2100, 1900, 2100, 2040],
        [1800, 1400, 1900, 1700, 1900, 2040]
    ]
    robot.execute_waypoints(waypoints, speed=150, acceleration=100)
"""

import os
import time
import json
import math
from typing import List, Dict, Tuple, Optional, Any

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *

# Control table addresses
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56
ADDR_SCS_PRESENT_LOAD      = 60
ADDR_SCS_PRESENT_VOLTAGE   = 62
ADDR_SCS_PRESENT_TEMPERATURE = 63

# Communication settings
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyACM1'
protocol_end            = 0

# Default servo configuration
DEFAULT_SERVOS = {
    "shoulder_pan": {"id": 1, "drive_mode": 0},
    "shoulder_lift": {"id": 2, "drive_mode": 0},
    "elbow_flex": {"id": 3, "drive_mode": 0},
    "wrist_flex": {"id": 4, "drive_mode": 0},
    "wrist_roll": {"id": 5, "drive_mode": 0},
    "gripper": {"id": 6, "drive_mode": 0}
}

# Calibration settings
CALIBRATION_FILE = "robot_calibration.json"
CALIBRATION_SPEED = 500          # Slow speed for calibration
CALIBRATION_ACC = 100            # Low acceleration for calibration
OVERLOAD_THRESHOLD = 3          # Number of consecutive overload errors to detect limit
POSITION_STEP = 10              # Step size during calibration
MAX_POSITION = 4095             # Maximum possible position value
MIN_POSITION = -4095            # Minimum possible position value (can be negative)

class Robot:
    """
    SO-100 Follower Robot Controller with Auto-Calibration and Waypoint Navigation
    """
    
    def __init__(self, device_name: str = DEVICENAME, baudrate: int = BAUDRATE):
        """
        Initialize the robot controller.
        
        Args:
            device_name: Serial device path (default: /dev/ttyACM1)
            baudrate: Communication baudrate (default: 1000000)
        """
        self.device_name = device_name
        self.baudrate = baudrate
        self.calibration = {}
        self.servo_names = {}  # Map ID to name
        self.port_handler = None
        self.packet_handler = None
        self.is_connected = False
        
        # Load existing calibration if available
        self.load_calibration()
    
    def connect(self) -> bool:
        """
        Connect to the servo bus.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.port_handler = PortHandler(self.device_name)
            self.packet_handler = PacketHandler(protocol_end)
            
            if not self.port_handler.openPort():
                print(f"Failed to open port {self.device_name}")
                return False
                
            if not self.port_handler.setBaudRate(self.baudrate):
                print(f"Failed to set baudrate to {self.baudrate}")
                return False
                
            self.is_connected = True
            print(f"✓ Connected to {self.device_name} at {self.baudrate} baud")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Safely disconnect from the servo bus."""
        if self.is_connected and self.port_handler:
            # Disable torque on all servos
            for servo_id in DEFAULT_SERVOS.values():
                try:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, servo_id["id"], ADDR_SCS_TORQUE_ENABLE, 0
                    )
                except:
                    pass
            self.port_handler.closePort()
            self.is_connected = False
            print("✓ Disconnected safely")
    
    def has_calibration(self) -> bool:
        """
        Check if calibration data is available.
        
        Returns:
            bool: True if calibration data exists, False otherwise
        """
        return len(self.calibration) > 0 and os.path.exists(CALIBRATION_FILE)
    
    def load_calibration(self) -> bool:
        """
        Load calibration data from file.
        
        Returns:
            bool: True if calibration loaded successfully, False otherwise
        """
        try:
            if not os.path.exists(CALIBRATION_FILE):
                print(f"No calibration file found at {CALIBRATION_FILE}")
                return False
                
            with open(CALIBRATION_FILE, 'r') as f:
                cal_data = json.load(f)
            
            self.calibration = {}
            self.servo_names = {}
            
            for name, data in cal_data.items():
                servo_id = data['id']
                self.calibration[servo_id] = {
                    'name': name,
                    'id': servo_id,
                    'drive_mode': data['drive_mode'],
                    'homing_offset': data['homing_offset'],
                    'range_min': data['range_min'],
                    'range_max': data['range_max'],
                    'range_size': data['range_max'] - data['range_min']
                }
                self.servo_names[servo_id] = name
            
            print(f"✓ Loaded calibration for {len(self.calibration)} servos")
            return True
            
        except Exception as e:
            print(f"Failed to load calibration: {e}")
            return False
    
    def save_calibration(self) -> bool:
        """
        Save calibration data to file.
        
        Returns:
            bool: True if calibration saved successfully, False otherwise
        """
        try:
            cal_data = {}
            for servo_id, cal in self.calibration.items():
                cal_data[cal['name']] = {
                    'id': cal['id'],
                    'drive_mode': cal['drive_mode'],
                    'homing_offset': cal['homing_offset'],
                    'range_min': cal['range_min'],
                    'range_max': cal['range_max']
                }
            
            with open(CALIBRATION_FILE, 'w') as f:
                json.dump(cal_data, f, indent=4)
            
            print(f"✓ Calibration saved to {CALIBRATION_FILE}")
            return True
            
        except Exception as e:
            print(f"Failed to save calibration: {e}")
            return False
    
    def setup_servo(self, servo_id: int, speed: int = 100, acceleration: int = 50) -> bool:
        """
        Setup a servo with specified parameters.
        
        Args:
            servo_id: ID of the servo
            speed: Movement speed (0-1000)
            acceleration: Movement acceleration (0-255)
            
        Returns:
            bool: True if setup successful, False otherwise
        """
        try:
            # Set acceleration
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_ACC, acceleration
            )
            if result != COMM_SUCCESS or error != 0:
                return False
                
            # Set speed
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_SPEED, speed
            )
            if result != COMM_SUCCESS or error != 0:
                return False
                
            # Enable torque
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_TORQUE_ENABLE, 1
            )
            if result != COMM_SUCCESS or error != 0:
                return False
                
            return True
            
        except Exception as e:
            print(f"Setup failed for servo {servo_id}: {e}")
            return False
    
    def get_servo_status(self, servo_id: int) -> Dict[str, Any]:
        """
        Get comprehensive status of a servo.
        
        Args:
            servo_id: ID of the servo
            
        Returns:
            dict: Servo status including position, speed, load, voltage, temperature
        """
        status = {
            'position': None,
            'speed': None,
            'load': None,
            'voltage': None,
            'temperature': None,
            'error': None
        }
        
        try:
            # Read position and speed
            pos_speed, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_POSITION
            )
            if result == COMM_SUCCESS and error == 0:
                status['position'] = SCS_LOWORD(pos_speed)
                status['speed'] = SCS_TOHOST(SCS_HIWORD(pos_speed), 15)
            else:
                status['error'] = self.packet_handler.getRxPacketError(error) if error != 0 else self.packet_handler.getTxRxResult(result)
            
            # Read load
            load, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_LOAD
            )
            if result == COMM_SUCCESS and error == 0:
                status['load'] = SCS_TOHOST(load, 10)
            
            # Read voltage
            voltage, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_VOLTAGE
            )
            if result == COMM_SUCCESS and error == 0:
                status['voltage'] = voltage / 10.0  # Convert to volts
            
            # Read temperature
            temp, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_TEMPERATURE
            )
            if result == COMM_SUCCESS and error == 0:
                status['temperature'] = temp
                
        except Exception as e:
            status['error'] = str(e)
        
        return status
    
    def move_servo(self, servo_id: int, position: int) -> bool:
        """
        Move servo to specified position.
        
        Args:
            servo_id: ID of the servo
            position: Target position
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        try:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION, position
            )
            
            if result != COMM_SUCCESS:
                return False
            elif error != 0:
                error_msg = self.packet_handler.getRxPacketError(error)
                if "Overload error" in error_msg:
                    return False  # Overload detected
                print(f"Warning for servo {servo_id}: {error_msg}")
                return False
            
            return True
            
        except Exception as e:
            print(f"Exception moving servo {servo_id}: {e}")
            return False
    
    def detect_joint_limits(self, servo_id: int, servo_name: str) -> Tuple[int, int]:
        """
        Detect the physical limits of a joint by moving until overload errors occur.
        
        Args:
            servo_id: ID of the servo
            servo_name: Name of the servo
            
        Returns:
            tuple: (min_position, max_position) or (None, None) if detection failed
        """
        print(f"\n--- Calibrating {servo_name} (ID {servo_id}) ---")
        
        # Setup servo for calibration
        if not self.setup_servo(servo_id, CALIBRATION_SPEED, CALIBRATION_ACC):
            print(f"✗ Failed to setup {servo_name}")
            return None, None
        
        # Get current position as starting point
        status = self.get_servo_status(servo_id)
        if status['position'] is None:
            print(f"✗ Cannot read position for {servo_name}")
            return None, None
        
        start_position = status['position']
        print(f"Starting position: {start_position}")
        
        # Find minimum limit (move towards negative values)
        print("Finding minimum limit...")
        min_position = start_position
        overload_count = 0
        
        for pos in range(start_position, MIN_POSITION - 1, -POSITION_STEP):
            if not self.move_servo(servo_id, pos):
                overload_count += 1
                if overload_count >= OVERLOAD_THRESHOLD:
                    min_position = pos + POSITION_STEP  # Last successful position
                    print(f"✓ Minimum limit detected at: {min_position}")
                    break
            else:
                overload_count = 0
                time.sleep(0.1)  # Small delay between moves
        else:
            min_position = MIN_POSITION
            print(f"✓ Reached absolute minimum: {min_position}")
        
        # Return to start position
        time.sleep(0.5)
        self.move_servo(servo_id, start_position)
        time.sleep(1)
        
        # Find maximum limit (move towards 4095)
        print("Finding maximum limit...")
        max_position = start_position
        overload_count = 0
        
        for pos in range(start_position, MAX_POSITION + 1, POSITION_STEP):
            if not self.move_servo(servo_id, pos):
                overload_count += 1
                if overload_count >= OVERLOAD_THRESHOLD:
                    max_position = pos - POSITION_STEP  # Last successful position
                    print(f"✓ Maximum limit detected at: {max_position}")
                    break
            else:
                overload_count = 0
                time.sleep(0.1)  # Small delay between moves
        else:
            max_position = MAX_POSITION
            print(f"✓ Reached absolute maximum: {max_position}")
        
        # Return to start position
        time.sleep(0.5)
        self.move_servo(servo_id, start_position)
        time.sleep(0.5)
        
        print(f"✓ {servo_name} limits: {min_position} to {max_position} (range: {max_position - min_position})")
        return min_position, max_position
    
    def manual_calibrate(self) -> bool:
        """
        Manual calibration by letting user move joints freely while recording ranges.
        
        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self.is_connected:
            print("✗ Not connected to robot")
            return False
        
        print("\n" + "="*60)
        print("MANUAL ROBOT CALIBRATION")
        print("="*60)
        print("This will disable torque on all joints so you can move them freely.")
        print("The system will record the minimum and maximum positions for each joint.")
        print("Move each joint through its full range of motion.")
        print()
        
        # Disable torque on all servos
        print("Disabling torque on all servos...")
        for servo_name, servo_config in DEFAULT_SERVOS.items():
            servo_id = servo_config["id"]
            try:
                result, error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, servo_id, ADDR_SCS_TORQUE_ENABLE, 0
                )
                if result == COMM_SUCCESS and error == 0:
                    print(f"✓ {servo_name} (ID {servo_id}) torque disabled")
                else:
                    print(f"⚠ {servo_name} (ID {servo_id}) may not be responding")
            except:
                print(f"⚠ {servo_name} (ID {servo_id}) may not be responding")
        
        print("\n" + "="*60)
        print("MOVE JOINTS THROUGH FULL RANGE")
        print("="*60)
        print("Now move each joint through its complete range of motion.")
        print("The system will continuously update the min/max values.")
        print("Press ENTER when you're done moving all joints...")
        print()
        
        # Initialize tracking variables
        joint_ranges = {}
        for servo_name, servo_config in DEFAULT_SERVOS.items():
            servo_id = servo_config["id"]
            joint_ranges[servo_id] = {
                'name': servo_name,
                'min_pos': None,
                'max_pos': None,
                'current_pos': None
            }
        
        # Continuous monitoring loop
        try:
            import select
            import sys
            
            print("Joint Positions (move joints to see updates):")
            print("-" * 80)
            
            while True:
                # Check if user pressed enter
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    input()  # Consume the enter
                    break
                
                # Read current positions and update ranges
                updated = False
                for servo_id, range_data in joint_ranges.items():
                    status = self.get_servo_status(servo_id)
                    if status['position'] is not None:
                        current_pos = status['position']
                        range_data['current_pos'] = current_pos
                        
                        # Update min/max
                        if range_data['min_pos'] is None or current_pos < range_data['min_pos']:
                            range_data['min_pos'] = current_pos
                            updated = True
                        if range_data['max_pos'] is None or current_pos > range_data['max_pos']:
                            range_data['max_pos'] = current_pos
                            updated = True
                
                # Print updated ranges if something changed
                if updated:
                    print("\033[H\033[J", end="")  # Clear screen
                    print("Joint Positions (move joints to see updates):")
                    print("-" * 80)
                    for servo_id in sorted(joint_ranges.keys()):
                        range_data = joint_ranges[servo_id]
                        min_pos = range_data['min_pos'] if range_data['min_pos'] is not None else "---"
                        max_pos = range_data['max_pos'] if range_data['max_pos'] is not None else "---"
                        current = range_data['current_pos'] if range_data['current_pos'] is not None else "---"
                        range_size = (range_data['max_pos'] - range_data['min_pos']) if (range_data['min_pos'] is not None and range_data['max_pos'] is not None) else "---"
                        print(f"{range_data['name']:12} (ID {servo_id}): Current={current:>5} | Min={min_pos:>5} | Max={max_pos:>5} | Range={range_size}")
                    print("\nPress ENTER when done moving all joints...")
                
                time.sleep(0.1)  # Small delay
                
        except KeyboardInterrupt:
            print("\nCalibration interrupted")
            return False
        
        print("\n" + "="*60)
        print("SET HOME POSITION")
        print("="*60)
        print("Now move the robot to its HOME position.")
        print("This should be a comfortable middle position for all joints.")
        print("Press ENTER when the robot is in the home position...")
        
        input()  # Wait for user to set home position
        
        # Record home positions
        home_positions = {}
        print("\nRecording home positions:")
        for servo_id, range_data in joint_ranges.items():
            status = self.get_servo_status(servo_id)
            if status['position'] is not None:
                home_positions[servo_id] = status['position']
                print(f"✓ {range_data['name']} home position: {status['position']}")
            else:
                print(f"✗ Could not read {range_data['name']} position")
                return False
        
        # Build calibration data
        self.calibration = {}
        self.servo_names = {}
        
        for servo_name, servo_config in DEFAULT_SERVOS.items():
            servo_id = servo_config["id"]
            range_data = joint_ranges[servo_id]
            
            if (range_data['min_pos'] is not None and 
                range_data['max_pos'] is not None and 
                servo_id in home_positions):
                
                # Calculate homing offset
                home_pos = home_positions[servo_id]
                homing_offset = home_pos - 2048  # Offset from theoretical center
                
                # Store calibration data
                self.calibration[servo_id] = {
                    'name': servo_name,
                    'id': servo_id,
                    'drive_mode': servo_config["drive_mode"],
                    'homing_offset': homing_offset,
                    'range_min': range_data['min_pos'],
                    'range_max': range_data['max_pos'],
                    'range_size': range_data['max_pos'] - range_data['min_pos']
                }
                self.servo_names[servo_id] = servo_name
                
                print(f"✓ {servo_name} calibrated: {range_data['min_pos']} to {range_data['max_pos']} (home: {home_pos})")
            else:
                print(f"✗ Insufficient data for {servo_name}")
        
        if len(self.calibration) > 0:
            # Save calibration to file
            if self.save_calibration():
                print(f"\n✓ Manual calibration complete! Calibrated {len(self.calibration)} servos")
                return True
            else:
                print("\n✗ Failed to save calibration")
                return False
        else:
            print("\n✗ No servos were successfully calibrated")
            return False
    
    def auto_calibrate(self) -> bool:
        """
        Automatically calibrate all servos by detecting their physical limits.
        
        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self.is_connected:
            print("✗ Not connected to robot")
            return False
        
        print("\n" + "="*60)
        print("AUTOMATIC ROBOT CALIBRATION")
        print("="*60)
        print("This will move each joint to find its physical limits.")
        print("The robot will detect overload errors to determine safe ranges.")
        print()
        
        response = input("Continue with auto-calibration? (y/N): ").strip().lower()
        if response != 'y':
            print("Calibration cancelled")
            return False
        
        self.calibration = {}
        self.servo_names = {}
        
        for servo_name, servo_config in DEFAULT_SERVOS.items():
            servo_id = servo_config["id"]
            
            # Detect limits for this servo
            min_pos, max_pos = self.detect_joint_limits(servo_id, servo_name)
            
            if min_pos is not None and max_pos is not None:
                # Calculate homing offset (center position)
                center_pos = (min_pos + max_pos) // 2
                homing_offset = center_pos - 2048  # Offset from theoretical center
                
                # Store calibration data
                self.calibration[servo_id] = {
                    'name': servo_name,
                    'id': servo_id,
                    'drive_mode': servo_config["drive_mode"],
                    'homing_offset': homing_offset,
                    'range_min': min_pos,
                    'range_max': max_pos,
                    'range_size': max_pos - min_pos
                }
                self.servo_names[servo_id] = servo_name
                
                print(f"✓ {servo_name} calibrated successfully")
            else:
                print(f"✗ Failed to calibrate {servo_name}")
        
        if len(self.calibration) > 0:
            # Save calibration to file
            if self.save_calibration():
                print(f"\n✓ Auto-calibration complete! Calibrated {len(self.calibration)} servos")
                return True
            else:
                print("\n✗ Failed to save calibration")
                return False
        else:
            print("\n✗ No servos were successfully calibrated")
            return False
    
    def calibrate(self) -> bool:
        """
        Main calibration method that offers choice between manual and auto calibration.
        
        Returns:
            bool: True if calibration successful, False otherwise
        """
        print("\n" + "="*60)
        print("ROBOT CALIBRATION")
        print("="*60)
        print("Choose calibration method:")
        print("1. Manual Calibration (Default) - Move joints by hand")
        print("2. Automatic Calibration - Robot moves to find limits")
        print()
        
        while True:
            response = input("Select calibration method (1/2) [1]: ").strip()
            if response == "" or response == "1":
                return self.manual_calibrate()
            elif response == "2":
                return self.auto_calibrate()
            else:
                print("Please enter 1 or 2")
                continue
    
    def move_to_position(self, positions: List[int], speed: int = 100, acceleration: int = 50) -> bool:
        """
        Move all servos to specified positions simultaneously.
        
        Args:
            positions: List of positions for each servo (by ID order 1-6)
            speed: Movement speed (0-1000)
            acceleration: Movement acceleration (0-255)
            
        Returns:
            bool: True if all movements initiated successfully, False otherwise
        """
        if not self.has_calibration():
            print("✗ No calibration data available")
            return False
        
        if len(positions) != len(self.calibration):
            print(f"✗ Expected {len(self.calibration)} positions, got {len(positions)}")
            return False
        
        # Setup all servos with new speed/acceleration
        for servo_id in self.calibration.keys():
            if not self.setup_servo(servo_id, speed, acceleration):
                print(f"✗ Failed to setup servo {servo_id}")
                return False
        
        # Send position commands to all servos
        success = True
        for i, servo_id in enumerate(sorted(self.calibration.keys())):
            cal = self.calibration[servo_id]
            target_pos = positions[i]
            
            # Clamp position to safe range
            safe_pos = max(cal['range_min'], min(cal['range_max'], target_pos))
            
            if safe_pos != target_pos:
                print(f"⚠ Clamped {cal['name']}: {target_pos} → {safe_pos}")
            
            if not self.move_servo(servo_id, safe_pos):
                print(f"✗ Failed to move {cal['name']}")
                success = False
        
        return success
    
    def execute_waypoints(self, waypoints: List[List[int]], speed: int = 100, 
                         acceleration: int = 50, wait_time: float = 2.0) -> bool:
        """
        Execute a sequence of waypoints.
        
        Args:
            waypoints: List of waypoints, each containing positions for all servos
            speed: Movement speed (0-1000)
            acceleration: Movement acceleration (0-255)
            wait_time: Time to wait at each waypoint (seconds)
            
        Returns:
            bool: True if all waypoints executed successfully, False otherwise
        """
        if not waypoints:
            print("✗ No waypoints provided")
            return False
        
        print(f"\n--- Executing {len(waypoints)} waypoints ---")
        print(f"Speed: {speed}, Acceleration: {acceleration}, Wait time: {wait_time}s")
        
        for i, waypoint in enumerate(waypoints):
            print(f"\nWaypoint {i+1}/{len(waypoints)}: {waypoint}")
            
            if not self.move_to_position(waypoint, speed, acceleration):
                print(f"✗ Failed to reach waypoint {i+1}")
                return False
            
            # Wait for movement to complete
            time.sleep(wait_time)
            
            # Show current positions
            print("Current positions:")
            for servo_id in sorted(self.calibration.keys()):
                cal = self.calibration[servo_id]
                status = self.get_servo_status(servo_id)
                if status['position'] is not None:
                    print(f"  {cal['name']}: {status['position']}")
                else:
                    print(f"  {cal['name']}: Error reading position")
        
        print("\n✓ All waypoints executed successfully")
        return True
    
    def get_current_positions(self) -> List[int]:
        """
        Get current positions of all servos.
        
        Returns:
            list: Current positions of all servos (by ID order 1-6)
        """
        positions = []
        for servo_id in sorted(self.calibration.keys()):
            status = self.get_servo_status(servo_id)
            positions.append(status['position'] if status['position'] is not None else 0)
        return positions
    
    def print_status(self):
        """Print comprehensive status of all servos."""
        if not self.has_calibration():
            print("No calibration data available")
            return
        
        print("\n--- Robot Status ---")
        for servo_id in sorted(self.calibration.keys()):
            cal = self.calibration[servo_id]
            status = self.get_servo_status(servo_id)
            
            print(f"{cal['name']} (ID {servo_id}):")
            if status['position'] is not None:
                range_pct = ((status['position'] - cal['range_min']) / cal['range_size']) * 100
                print(f"  Position: {status['position']} ({range_pct:.1f}% of range)")
                print(f"  Speed: {status['speed']}")
                print(f"  Load: {status['load']}")
                print(f"  Voltage: {status['voltage']}V")
                print(f"  Temperature: {status['temperature']}°C")
            else:
                print(f"  Error: {status['error']}")
            print()


def main():
    """Example usage of the Robot class."""
    robot = Robot()
    
    try:
        # Connect to robot
        if not robot.connect():
            print("Failed to connect to robot")
            return
        
        # Check if calibration exists
        if not robot.has_calibration():
            print("\nNo calibration found.")
            response = input("Would you like to calibrate the robot? (y/N): ").strip().lower()
            if response == 'y':
                if not robot.calibrate():
                    print("Calibration failed")
                    return
            else:
                print("Cannot proceed without calibration")
                return
        
        # Show current status
        robot.print_status()
        
        # Full range of motion test - one joint at a time
        print("\nFull Range of Motion Test:")
        current_pos = robot.get_current_positions()
        print(f"Starting position: {current_pos}")
        
        response = input("Test full range of motion (one joint at a time) at speed 200? (y/N): ").strip().lower()
        if response == 'y':
            # Test each joint through its full range while keeping others at center
            for i, servo_id in enumerate(sorted(robot.calibration.keys())):
                cal = robot.calibration[servo_id]
                print(f"\n--- Testing {cal['name']} (Joint {i+1}) ---")
                
                # Calculate center positions for all other joints
                center_positions = []
                for j, other_servo_id in enumerate(sorted(robot.calibration.keys())):
                    other_cal = robot.calibration[other_servo_id]
                    center_pos = (other_cal['range_min'] + other_cal['range_max']) // 2
                    center_positions.append(center_pos)
                
                # Create waypoints for this joint test
                waypoints = []
                
                # 1. Move all joints to center
                waypoints.append(center_positions.copy())
                
                # 2. Move test joint to minimum while others stay at center
                min_waypoint = center_positions.copy()
                min_waypoint[i] = cal['range_min']
                waypoints.append(min_waypoint)
                
                # 3. Move test joint to maximum while others stay at center
                max_waypoint = center_positions.copy()
                max_waypoint[i] = cal['range_max']
                waypoints.append(max_waypoint)
                
                # 4. Move test joint through 5 intermediate positions
                for step in range(1, 6):  # 20%, 40%, 60%, 80%, 100% of range
                    progress = step / 6.0
                    intermediate_pos = int(cal['range_min'] + (cal['range_size'] * progress))
                    intermediate_waypoint = center_positions.copy()
                    intermediate_waypoint[i] = intermediate_pos
                    waypoints.append(intermediate_waypoint)
                
                # 5. Return to center
                waypoints.append(center_positions.copy())
                
                print(f"Testing {cal['name']} through {len(waypoints)} waypoints:")
                print(f"  Range: {cal['range_min']} to {cal['range_max']} ({cal['range_size']} steps)")
                
                # Execute waypoints for this joint
                success = robot.execute_waypoints(
                    waypoints, 
                    speed=200, 
                    acceleration=100, 
                    wait_time=2.0
                )
                
                if success:
                    print(f"✓ {cal['name']} full range test completed successfully")
                else:
                    print(f"✗ {cal['name']} full range test failed")
                    break
                
                # Wait between joint tests
                print("Waiting 3 seconds before next joint test...")
                time.sleep(3)
            
            # Return to original starting position
            print(f"\n--- Returning to original starting position ---")
            robot.move_to_position(current_pos, speed=200, acceleration=100)
            time.sleep(2)
            print("✓ Full range of motion test complete!")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()