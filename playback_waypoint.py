#!/usr/bin/env python3

"""
Waypoint Playback Tool for SO-100 Follower Robot
================================================

This tool plays back recorded waypoints from JSON files created by record_waypoint.py

Usage:
    python playback_waypoint.py [waypoint_file] [options]
    
Options:
    --speed SPEED           Movement speed (default: 200)
    --acceleration ACC      Movement acceleration (default: 100)  
    --wait-time TIME        Wait time at each waypoint in seconds (default: 2.0)
    --loop COUNT            Number of times to loop playback (default: 1, 0=infinite)
    --start-waypoint N      Start from waypoint N (default: 1)
    --end-waypoint N        End at waypoint N (default: last)
    --reverse               Play waypoints in reverse order
    --list                  List available waypoint files and exit

Examples:
    python playback_waypoint.py waypoints_20241201_143022.json
    python playback_waypoint.py my_waypoints.json --speed 150 --wait-time 3.0
    python playback_waypoint.py demo.json --loop 3 --reverse
    python playback_waypoint.py --list
"""

import os
import time
import json
import sys
import argparse
import glob
from typing import List, Dict, Any, Optional

from robot import Robot

class WaypointPlayer:
    """
    Plays back recorded robot waypoints from JSON files.
    """
    
    def __init__(self, robot: Robot):
        """
        Initialize the waypoint player.
        
        Args:
            robot: Connected Robot instance
        """
        self.robot = robot
        self.waypoint_data = None
        self.waypoints = []
        
    def list_waypoint_files(self) -> List[str]:
        """
        List all available waypoint JSON files in current directory.
        
        Returns:
            list: List of waypoint filenames
        """
        waypoint_files = glob.glob("waypoints_*.json") + glob.glob("*waypoint*.json")
        return sorted(waypoint_files)
    
    def load_waypoints(self, filename: str) -> bool:
        """
        Load waypoints from JSON file.
        
        Args:
            filename: Path to waypoint JSON file
            
        Returns:
            bool: True if loaded successfully, False otherwise
        """
        try:
            if not os.path.exists(filename):
                print(f"✗ Waypoint file not found: {filename}")
                return False
            
            with open(filename, 'r') as f:
                self.waypoint_data = json.load(f)
            
            # Validate file structure
            if 'waypoints' not in self.waypoint_data:
                print(f"✗ Invalid waypoint file format: missing 'waypoints' key")
                return False
            
            self.waypoints = self.waypoint_data['waypoints']
            
            if not self.waypoints:
                print(f"✗ No waypoints found in file: {filename}")
                return False
            
            # Display file info
            metadata = self.waypoint_data.get('metadata', {})
            print(f"✓ Loaded waypoint file: {filename}")
            print(f"  Robot type: {metadata.get('robot_type', 'unknown')}")
            print(f"  Created: {metadata.get('created_at', 'unknown')}")
            print(f"  Total waypoints: {len(self.waypoints)}")
            print(f"  Joint names: {', '.join(metadata.get('joint_names', []))}")
            
            return True
            
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON format in {filename}: {e}")
            return False
        except Exception as e:
            print(f"✗ Failed to load waypoints from {filename}: {e}")
            return False
    
    def validate_waypoints(self) -> bool:
        """
        Validate that waypoints are compatible with current robot calibration.
        
        Returns:
            bool: True if waypoints are valid, False otherwise
        """
        if not self.robot.has_calibration():
            print("✗ Robot not calibrated")
            return False
        
        if not self.waypoints:
            print("✗ No waypoints loaded")
            return False
        
        # Check if number of joints matches
        expected_joints = len(self.robot.calibration)
        
        for i, waypoint in enumerate(self.waypoints):
            positions = waypoint.get('positions', [])
            if len(positions) != expected_joints:
                print(f"✗ Waypoint {i+1} has {len(positions)} positions, expected {expected_joints}")
                return False
        
        print(f"✓ All {len(self.waypoints)} waypoints validated")
        return True
    
    def preview_waypoints(self, start_idx: int = 0, end_idx: Optional[int] = None, reverse: bool = False):
        """
        Preview waypoints that will be played.
        
        Args:
            start_idx: Starting waypoint index (0-based)
            end_idx: Ending waypoint index (0-based, inclusive)
            reverse: Whether to reverse the order
        """
        if not self.waypoints:
            print("No waypoints to preview")
            return
        
        if end_idx is None:
            end_idx = len(self.waypoints) - 1
        
        # Get waypoints to play
        waypoints_to_play = self.waypoints[start_idx:end_idx+1]
        if reverse:
            waypoints_to_play = waypoints_to_play[::-1]
        
        print(f"\nWaypoint Preview ({len(waypoints_to_play)} waypoints):")
        print("-" * 80)
        
        for i, waypoint in enumerate(waypoints_to_play):
            original_id = waypoint.get('id', i+1)
            positions = waypoint.get('positions', [])
            timestamp = waypoint.get('timestamp', 'unknown')
            
            print(f"Step {i+1:2d} (Original ID {original_id}): {positions}")
            if len(waypoints_to_play) <= 10:  # Only show timestamps for small lists
                print(f"         Recorded: {timestamp}")
    
    def playback_waypoints(self, speed: int = 200, acceleration: int = 100, 
                          wait_time: float = 2.0, loop_count: int = 1,
                          start_waypoint: int = 1, end_waypoint: Optional[int] = None,
                          reverse: bool = False) -> bool:
        """
        Play back the loaded waypoints.
        
        Args:
            speed: Movement speed (0-1000)
            acceleration: Movement acceleration (0-255)
            wait_time: Time to wait at each waypoint (seconds)
            loop_count: Number of times to loop (0 = infinite)
            start_waypoint: Starting waypoint number (1-based)
            end_waypoint: Ending waypoint number (1-based, inclusive)
            reverse: Play waypoints in reverse order
            
        Returns:
            bool: True if playback completed successfully, False otherwise
        """
        if not self.validate_waypoints():
            return False
        
        # Convert to 0-based indices
        start_idx = start_waypoint - 1
        end_idx = (end_waypoint - 1) if end_waypoint else len(self.waypoints) - 1
        
        # Validate indices
        if start_idx < 0 or start_idx >= len(self.waypoints):
            print(f"✗ Invalid start waypoint: {start_waypoint}")
            return False
        
        if end_idx < 0 or end_idx >= len(self.waypoints) or end_idx < start_idx:
            print(f"✗ Invalid end waypoint: {end_waypoint}")
            return False
        
        # Get waypoints to play
        waypoints_to_play = self.waypoints[start_idx:end_idx+1]
        if reverse:
            waypoints_to_play = waypoints_to_play[::-1]
        
        print(f"\n" + "="*60)
        print("WAYPOINT PLAYBACK")
        print("="*60)
        print(f"Waypoints: {len(waypoints_to_play)} (from {start_waypoint} to {end_waypoint or len(self.waypoints)})")
        print(f"Speed: {speed}, Acceleration: {acceleration}, Wait time: {wait_time}s")
        print(f"Loop count: {'infinite' if loop_count == 0 else loop_count}")
        print(f"Reverse order: {reverse}")
        
        # Preview waypoints
        self.preview_waypoints(start_idx, end_idx, reverse)
        
        # Confirm playback
        response = input(f"\nStart playback? (y/N): ").strip().lower()
        if response != 'y':
            print("Playback cancelled")
            return False
        
        # Execute playback
        try:
            current_loop = 0
            while loop_count == 0 or current_loop < loop_count:
                current_loop += 1
                
                if loop_count == 0:
                    print(f"\n--- Loop {current_loop} (infinite mode, Ctrl+C to stop) ---")
                elif loop_count > 1:
                    print(f"\n--- Loop {current_loop}/{loop_count} ---")
                
                # Execute waypoints
                success = self.robot.execute_waypoints(
                    [wp['positions'] for wp in waypoints_to_play],
                    speed=speed,
                    acceleration=acceleration,
                    wait_time=wait_time
                )
                
                if not success:
                    print(f"✗ Playback failed during loop {current_loop}")
                    return False
                
                if loop_count == 0 or current_loop < loop_count:
                    print(f"Loop {current_loop} completed. Waiting 2 seconds before next loop...")
                    time.sleep(2)
            
            print(f"\n✓ Waypoint playback completed successfully!")
            return True
            
        except KeyboardInterrupt:
            print(f"\n\nPlayback interrupted by user during loop {current_loop}")
            return False


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Play back recorded robot waypoints",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python playback_waypoint.py waypoints_20241201_143022.json
  python playback_waypoint.py my_waypoints.json --speed 150 --wait-time 3.0
  python playback_waypoint.py demo.json --loop 3 --reverse
  python playback_waypoint.py --list
        """
    )
    
    parser.add_argument('waypoint_file', nargs='?', help='Waypoint JSON file to play back')
    parser.add_argument('--speed', type=int, default=200, help='Movement speed (default: 200)')
    parser.add_argument('--acceleration', type=int, default=100, help='Movement acceleration (default: 100)')
    parser.add_argument('--wait-time', type=float, default=2.0, help='Wait time at each waypoint in seconds (default: 2.0)')
    parser.add_argument('--loop', type=int, default=1, help='Number of times to loop playback (default: 1, 0=infinite)')
    parser.add_argument('--start-waypoint', type=int, default=1, help='Start from waypoint N (default: 1)')
    parser.add_argument('--end-waypoint', type=int, help='End at waypoint N (default: last)')
    parser.add_argument('--reverse', action='store_true', help='Play waypoints in reverse order')
    parser.add_argument('--list', action='store_true', help='List available waypoint files and exit')
    
    return parser.parse_args()


def main():
    """Main function for waypoint playback."""
    args = parse_arguments()
    
    print("SO-100 Follower Robot - Waypoint Player")
    print("=" * 42)
    
    # Handle --list option
    if args.list:
        player = WaypointPlayer(None)  # Don't need robot connection for listing
        waypoint_files = player.list_waypoint_files()
        
        if waypoint_files:
            print(f"\nAvailable waypoint files ({len(waypoint_files)}):")
            print("-" * 50)
            for i, filename in enumerate(waypoint_files, 1):
                file_size = os.path.getsize(filename)
                mod_time = time.ctime(os.path.getmtime(filename))
                print(f"{i:2d}. {filename} ({file_size} bytes, {mod_time})")
        else:
            print("\nNo waypoint files found in current directory")
            print("Waypoint files should match patterns: waypoints_*.json or *waypoint*.json")
        
        return
    
    # Determine waypoint file
    waypoint_file = args.waypoint_file
    
    if not waypoint_file:
        # Try to find the most recent waypoint file
        player = WaypointPlayer(None)
        waypoint_files = player.list_waypoint_files()
        
        if waypoint_files:
            waypoint_file = waypoint_files[-1]  # Most recent
            print(f"No waypoint file specified. Using most recent: {waypoint_file}")
        else:
            print("✗ No waypoint file specified and no waypoint files found")
            print("Usage: python playback_waypoint.py <waypoint_file>")
            print("Or use --list to see available files")
            return
    
    # Initialize robot and player
    robot = Robot()
    player = WaypointPlayer(robot)
    
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
        
        # Load waypoints
        if not player.load_waypoints(waypoint_file):
            return
        
        # Start playback
        success = player.playback_waypoints(
            speed=args.speed,
            acceleration=args.acceleration,
            wait_time=args.wait_time,
            loop_count=args.loop,
            start_waypoint=args.start_waypoint,
            end_waypoint=args.end_waypoint,
            reverse=args.reverse
        )
        
        if success:
            print("\n✓ Waypoint playback completed successfully!")
        else:
            print("\n✗ Waypoint playback failed")
            
    except KeyboardInterrupt:
        print("\n\nPlayback interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main() 