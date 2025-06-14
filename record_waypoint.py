#!/usr/bin/env python3

"""
Waypoint Recording Tool for SO-100 Follower Robot
=================================================

This tool allows you to record waypoints by moving the robot to desired positions
and pressing ENTER to record each waypoint. Type 'exit' to finish recording.

Usage:
    python record_waypoint.py [waypoint_filename]
    
    If no filename is provided, it will prompt for one.
"""

import os
import time
import json
import sys
from datetime import datetime
from typing import List, Dict, Any

from robot import Robot

class WaypointRecorder:
    """
    Records robot waypoints by capturing current positions when user presses enter.
    """
    
    def __init__(self, robot: Robot):
        """
        Initialize the waypoint recorder.
        
        Args:
            robot: Connected Robot instance
        """
        self.robot = robot
        self.waypoints = []
        self.waypoint_file = None
        
    def get_waypoint_filename(self) -> str:
        """
        Get waypoint filename from user input or generate default.
        
        Returns:
            str: Waypoint filename
        """
        if len(sys.argv) > 1:
            filename = sys.argv[1]
        else:
            print("\nWaypoint filename options:")
            print("1. Enter custom filename")
            print("2. Use default timestamp-based filename")
            
            choice = input("Choose option (1/2) [2]: ").strip()
            
            if choice == "1":
                filename = input("Enter waypoint filename (without .json): ").strip()
                if not filename:
                    filename = f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            else:
                filename = f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Ensure .json extension
        if not filename.endswith('.json'):
            filename += '.json'
            
        return filename
    
    def display_current_position(self) -> List[int]:
        """
        Display current robot position and return position list.
        
        Returns:
            list: Current positions of all servos
        """
        if not self.robot.has_calibration():
            print("✗ No calibration data available")
            return []
        
        positions = []
        print("\nCurrent Robot Position:")
        print("-" * 50)
        
        for servo_id in sorted(self.robot.calibration.keys()):
            cal = self.robot.calibration[servo_id]
            status = self.robot.get_servo_status(servo_id)
            
            if status['position'] is not None:
                position = status['position']
                positions.append(position)
                
                # Calculate percentage of range
                range_pct = ((position - cal['range_min']) / cal['range_size']) * 100
                
                print(f"{cal['name']:12} (ID {servo_id}): {position:>5} ({range_pct:>5.1f}% of range)")
            else:
                print(f"{cal['name']:12} (ID {servo_id}): ERROR - {status.get('error', 'Unknown')}")
                positions.append(0)  # Default value for failed reads
        
        return positions
    
    def record_waypoint(self) -> bool:
        """
        Record waypoints interactively.
        
        Returns:
            bool: True if recording completed successfully, False otherwise
        """
        if not self.robot.is_connected:
            print("✗ Robot not connected")
            return False
        
        if not self.robot.has_calibration():
            print("✗ Robot not calibrated")
            return False
        
        # Get filename
        self.waypoint_file = self.get_waypoint_filename()
        
        print("\n" + "="*60)
        print("WAYPOINT RECORDING MODE")
        print("="*60)
        print(f"Recording waypoints to: {self.waypoint_file}")
        print()
        print("Instructions:")
        print("1. Move the robot to desired position")
        print("2. Press ENTER to record the waypoint")
        print("3. Type 'exit' and press ENTER to finish recording")
        print("4. Type 'delete' and press ENTER to delete the last waypoint")
        print("5. Type 'show' and press ENTER to show all recorded waypoints")
        print()
        
        # Disable torque for manual positioning
        print("Disabling torque on all servos for manual positioning...")
        from robot import DEFAULT_SERVOS, ADDR_SCS_TORQUE_ENABLE, COMM_SUCCESS
        
        for servo_name, servo_config in DEFAULT_SERVOS.items():
            servo_id = servo_config["id"]
            try:
                result, error = self.robot.packet_handler.write1ByteTxRx(
                    self.robot.port_handler, servo_id, ADDR_SCS_TORQUE_ENABLE, 0
                )
                if result == COMM_SUCCESS and error == 0:
                    print(f"✓ {servo_name} torque disabled")
                else:
                    print(f"⚠ {servo_name} may not be responding")
            except:
                print(f"⚠ {servo_name} may not be responding")
        
        print("\n" + "="*60)
        print("READY TO RECORD WAYPOINTS")
        print("="*60)
        
        waypoint_count = 0
        
        while True:
            # Display current position
            current_positions = self.display_current_position()
            
            if not current_positions:
                print("✗ Could not read robot positions")
                return False
            
            print(f"\nWaypoints recorded: {len(self.waypoints)}")
            print("Commands: ENTER=record, 'exit'=finish, 'delete'=remove last, 'show'=list all")
            
            # Get user input
            user_input = input("\nPress ENTER to record waypoint (or type command): ").strip().lower()
            
            if user_input == 'exit':
                print(f"\nFinishing recording session...")
                break
            elif user_input == 'delete':
                if self.waypoints:
                    deleted = self.waypoints.pop()
                    print(f"✓ Deleted waypoint {len(self.waypoints) + 1}: {deleted['positions']}")
                else:
                    print("⚠ No waypoints to delete")
                continue
            elif user_input == 'show':
                self.show_waypoints()
                continue
            elif user_input == '':
                # Record waypoint
                waypoint_count += 1
                timestamp = datetime.now().isoformat()
                
                waypoint = {
                    'id': waypoint_count,
                    'timestamp': timestamp,
                    'positions': current_positions.copy(),
                    'joint_names': [self.robot.calibration[servo_id]['name'] 
                                  for servo_id in sorted(self.robot.calibration.keys())]
                }
                
                self.waypoints.append(waypoint)
                print(f"✓ Recorded waypoint {waypoint_count}: {current_positions}")
            else:
                print("⚠ Unknown command. Use ENTER, 'exit', 'delete', or 'show'")
        
        # Save waypoints to file
        return self.save_waypoints()
    
    def show_waypoints(self):
        """Display all recorded waypoints."""
        if not self.waypoints:
            print("No waypoints recorded yet.")
            return
        
        print(f"\nRecorded Waypoints ({len(self.waypoints)} total):")
        print("-" * 80)
        
        for i, waypoint in enumerate(self.waypoints):
            print(f"Waypoint {waypoint['id']:2d}: {waypoint['positions']}")
            if i < len(self.waypoints) - 1:  # Don't print separator after last waypoint
                print("             " + " " * len(str(waypoint['positions'])))
    
    def save_waypoints(self) -> bool:
        """
        Save recorded waypoints to JSON file.
        
        Returns:
            bool: True if saved successfully, False otherwise
        """
        if not self.waypoints:
            print("⚠ No waypoints to save")
            return True
        
        try:
            # Create waypoint data structure
            waypoint_data = {
                'metadata': {
                    'robot_type': 'so100_follower',
                    'created_at': datetime.now().isoformat(),
                    'total_waypoints': len(self.waypoints),
                    'joint_names': [self.robot.calibration[servo_id]['name'] 
                                  for servo_id in sorted(self.robot.calibration.keys())],
                    'calibration_file': 'robot_calibration.json'
                },
                'waypoints': self.waypoints
            }
            
            # Save to file
            with open(self.waypoint_file, 'w') as f:
                json.dump(waypoint_data, f, indent=4)
            
            print(f"\n✓ Saved {len(self.waypoints)} waypoints to {self.waypoint_file}")
            print(f"File size: {os.path.getsize(self.waypoint_file)} bytes")
            return True
            
        except Exception as e:
            print(f"✗ Failed to save waypoints: {e}")
            return False


def main():
    """Main function for waypoint recording."""
    print("SO-100 Follower Robot - Waypoint Recorder")
    print("=" * 45)
    
    robot = Robot()
    recorder = WaypointRecorder(robot)
    
    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Failed to connect to robot")
            return
        
        # Check calibration
        if not robot.has_calibration():
            print("✗ No calibration found. Please calibrate the robot first.")
            print("Run: python robot.py")
            return
        
        print(f"✓ Robot connected and calibrated ({len(robot.calibration)} servos)")
        
        # Start recording
        if recorder.record_waypoint():
            print("\n✓ Waypoint recording completed successfully!")
            print(f"Use 'python playback_waypoint.py {recorder.waypoint_file}' to play back")
        else:
            print("\n✗ Waypoint recording failed")
            
    except KeyboardInterrupt:
        print("\n\nRecording interrupted by user")
        if recorder.waypoints:
            save_choice = input("Save recorded waypoints? (y/N): ").strip().lower()
            if save_choice == 'y':
                if not recorder.waypoint_file:
                    recorder.waypoint_file = recorder.get_waypoint_filename()
                recorder.save_waypoints()
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main() 