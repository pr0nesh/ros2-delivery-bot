#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import threading
import time
import sys
import select
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class InputHandler:
    @staticmethod
    def check_input(timeout=0.1):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.readline().strip() if ready else None

class RestaurantRobot(Node):
    def __init__(self):
        super().__init__('restaurant_robot')
        self.declare_parameters('', [
            ('confirmation_timeout', 30),
            ('wait_time_between_orders', 5),
            ('nav_timeout', 90.0),
            ('locations_file', '')
        ])
        self.confirmation_timeout = self.get_parameter('confirmation_timeout').value
        self.wait_time_between_orders = self.get_parameter('wait_time_between_orders').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        
        # Default locations if file not provided
        self.locations = {
            'HOME': {"x": -1.0, "y": 0.0, "theta": 0.0},
            'KITCHEN': {"x": 1.0, "y": -0.5, "theta": 0.0},
            'TABLE1': {"x": 4.00, "y": 1.3, "theta": 0.0},
            'TABLE2': {"x": 4.00, "y": 0.0, "theta": 0.0},
            'TABLE3': {"x": 4.00, "y": -1.3, "theta": 0.0}
        }
        
        # Try loading custom locations
        locations_file = self.get_parameter('locations_file').value
        if locations_file:
            try:
                file_path = os.path.join(
                    get_package_share_directory('pranesh_description'), 
                    'config', locations_file
                )
                with open(file_path, 'r') as file:
                    self.locations = yaml.safe_load(file)
            except Exception as e:
                self.get_logger().warn(f'Error loading locations: {e}. Using defaults.')
        
        # Initialize state variables
        self.confirmation_received = False
        self.canceled_tables = []
        self.current_destination = None
        self.navigator_ready = False
        
        # Start navigator in background
        threading.Thread(target=self._initialize_navigation, daemon=True).start()
        
        self.get_logger().info('Robot starting. Type "help" for commands.')
        
    def _initialize_navigation(self):
        try:
            self.nav = BasicNavigator()
            
            # Initialize quaternions
            self.quaternions = [quaternion_from_euler(0.0, 0.0, angle) 
                               for angle in [0.0, 0.0, 0.0, 0.0]]
                
            # Set initial pose (home)
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = self.locations['HOME']["x"]
            initial_pose.pose.position.y = self.locations['HOME']["y"]
            initial_pose.pose.orientation.x = self.quaternions[0][0]
            initial_pose.pose.orientation.y = self.quaternions[0][1]
            initial_pose.pose.orientation.z = self.quaternions[0][2]
            initial_pose.pose.orientation.w = self.quaternions[0][3]
            
            self.nav.setInitialPose(initial_pose)
            self.nav.waitUntilNav2Active()
            
            self.navigator_ready = True
            self.get_logger().info('Navigation ready! Send robot with "t1" or "1,2,3"')
        except Exception as e:
            self.get_logger().error(f'Navigation initialization error: {e}')
    
    def _create_goal_pose(self, position):
        """Create a goal pose from position"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = position["x"]
        goal.pose.position.y = position["y"]
        goal.pose.orientation.x = self.quaternions[0][0]
        goal.pose.orientation.y = self.quaternions[0][1]
        goal.pose.orientation.z = self.quaternions[0][2]
        goal.pose.orientation.w = self.quaternions[0][3]
        return goal
    
    def wait_for_navigation_ready(self):
        """Wait until navigation is initialized"""
        if not self.navigator_ready:
            spinner_chars = ['|', '/', '-', '\\']
            i = 0
            while not self.navigator_ready:
                sys.stdout.write(f"\rInitializing {spinner_chars[i % len(spinner_chars)]} ")
                sys.stdout.flush()
                i += 1
                time.sleep(0.1)
            sys.stdout.write("\rNavigation ready!        \n")
        return True
    
    def go_home(self):
        """Immediately navigate to home position"""
        if not self.wait_for_navigation_ready():
            return TaskResult.FAILED
        
        # Cancel any active navigation task
        if not self.nav.isTaskComplete():
            self.nav.cancelTask()
            time.sleep(0.5)
        
        self.get_logger().info('Going HOME directly...')
        result = self.navigate_to('HOME', "Home", interactive=False)
        return result
        
    def navigate_to(self, location_key, location_name, interactive=True):
        """Navigate to a location with optional interactive mode"""
        if not self.wait_for_navigation_ready():
            return TaskResult.FAILED
            
        # Extract table number if this is a table
        target_table_num = None
        if "TABLE" in location_key:
            try:
                target_table_num = int(location_key.replace("TABLE", ""))
            except: pass
                
        self.get_logger().info(f'Going to {location_name}...')
        if interactive:
            print('Commands: "cl" to cancel and go home, "c1/c2/c3" to cancel specific table')
            
        # Set destination and start navigation
        self.current_destination = location_name
        goal_pose = self._create_goal_pose(self.locations[location_key])
        
        # Cancel any ongoing navigation
        if not self.nav.isTaskComplete():
            self.nav.cancelTask()
            time.sleep(0.5)
            
        self.nav.goToPose(goal_pose)
        
        # Monitor navigation progress
        last_status_time = time.time()
        start_time = time.time()
        
        print(f"Distance to {location_name}: calculating...")
        
        while not self.nav.isTaskComplete():
            # Check timeout for non-interactive mode
            if not interactive and time.time() - start_time > self.nav_timeout:
                self.get_logger().warning(f'Navigation timeout after {self.nav_timeout}s')
                self.nav.cancelTask()
                return TaskResult.FAILED
                
            # Check for user input in interactive mode
            if interactive:
                user_input = InputHandler.check_input(0.1)
                if user_input:
                    # Handle table-specific cancellation
                    if user_input.lower() in ['c1', 'c2', 'c3']:
                        canceled_table = int(user_input.lower()[1])
                        self.get_logger().info(f'Table {canceled_table} canceled!')
                        self.canceled_tables.append(canceled_table)
                        
                        # Cancel navigation if going to the canceled table
                        if target_table_num and target_table_num == canceled_table:
                            self.nav.cancelTask()
                            return TaskResult.CANCELED
                    
                    # Handle general navigation cancellation and go home
                    elif user_input.lower() == 'cl':
                        self.nav.cancelTask()
                        time.sleep(0.5)
                        self.go_home()
                        return TaskResult.CANCELED
            
            # Show distance periodically
            current_time = time.time()
            if current_time - last_status_time >= 1.0:
                last_status_time = current_time
                try:
                    feedback = self.nav.getFeedback()
                    if feedback:
                        print(f"Distance to {location_name}: {feedback.distance_remaining:.2f}m")
                except: pass
            
            time.sleep(0.05)
        
        # Get result
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Reached {location_name}!')
        else:
            self.get_logger().error(f'Failed to reach {location_name}!')
        return result
    
    def wait_for_confirmation(self, location):
        """Wait for confirmation with timeout"""
        # Extract table number if this is a table location
        table_num = None
        if "TABLE" in location.upper():
            try:
                table_num = int(location.upper().replace("TABLE", "").strip())
            except: pass
        
        print(f"Enter 'c' to confirm, 'cl' to go home (timeout: {self.confirmation_timeout}s)")
        if table_num:
            print(f"You can also cancel with 'c{table_num}'")
        
        start_time = time.time()
        while time.time() - start_time < self.confirmation_timeout:
            user_input = InputHandler.check_input(0.1)
            if user_input:
                if user_input.lower() == 'c':
                    return True
                elif user_input.lower() == 'cl':
                    # Go directly home when cl is entered
                    self.go_home()
                    return False
                elif table_num and user_input.lower() == f'c{table_num}':
                    self.canceled_tables.append(table_num)
                    return False
        return False
    
    def handle_orders(self, table_nums):
        """Handle deliveries to one or more tables"""
        if not self.wait_for_navigation_ready():
            return False
            
        # Ensure table_nums is a list
        if not isinstance(table_nums, list):
            table_nums = [table_nums]
            
        # Filter valid tables
        valid_orders = [t for t in table_nums 
                        if f'TABLE{t}' in self.locations and t not in self.canceled_tables]
                        
        if not valid_orders:
            self.get_logger().warning('No valid orders to process')
            return False
            
        self.get_logger().info(f'Processing orders for Tables: {valid_orders}')
        
        # First go to kitchen
        result = self.navigate_to('KITCHEN', "Kitchen")
        if result != TaskResult.SUCCEEDED:
            return False
            
        # Check for new cancellations
        valid_orders = [t for t in valid_orders if t not in self.canceled_tables]
        if not valid_orders:
            self.navigate_to('HOME', "Home", interactive=False)
            return False
            
        # Get kitchen confirmation
        if not self.wait_for_confirmation("Kitchen"):
            return False  # The go_home is already handled in wait_for_confirmation
            
        # Check again after kitchen confirmation
        valid_orders = [t for t in valid_orders if t not in self.canceled_tables]
        if not valid_orders:
            self.navigate_to('HOME', "Home", interactive=False)
            return False
            
        # Visit each table
        successful_tables = []
        failed_tables = []
        
        for table_num in valid_orders:
            # Skip canceled tables
            if table_num in self.canceled_tables:
                continue
                
            # Go to table
            table_key = f'TABLE{table_num}'
            result = self.navigate_to(table_key, f"Table {table_num}")
            
            # Check for cancellation during navigation
            if result != TaskResult.SUCCEEDED or table_num in self.canceled_tables:
                continue
                
            # Wait for confirmation
            if self.wait_for_confirmation(f"Table {table_num}"):
                successful_tables.append(table_num)
            else:
                if table_num not in self.canceled_tables:
                    failed_tables.append(table_num)
                    
        # Return path based on delivery results
        if failed_tables or self.canceled_tables:
            self.navigate_to('KITCHEN', "Kitchen", interactive=False)
            
        # Return home
        self.navigate_to('HOME', "Home", interactive=False)
        
        # Print summary
        print('\n== Delivery Summary ==')
        if successful_tables:
            print(f'Successfully delivered to Tables: {successful_tables}')
        if self.canceled_tables:
            print(f'Canceled deliveries for Tables: {self.canceled_tables}')
        if failed_tables:
            print(f'No confirmation from Tables: {failed_tables}')
            
        # Clear canceled tables for next run
        self.canceled_tables = []
        
        return len(successful_tables) > 0
    
    def process_command(self, command):
        """Process user commands"""
        if not command:
            return True
            
        # Special commands
        if command.lower() == 'q':
            return False
            
        if command.lower() == 'help':
            print("\nCOMMANDS:")
            print("- t1, t2, t3: Order for specific table")
            print("- 1,2,3 or t1,t2,t3: Multiple orders")
            print("- c: Confirm when prompted")
            print("- cl: Cancel current action and go home")
            print("- c1, c2, c3: Cancel specific table")
            print("- home: Go directly home")
            print("- status: Show robot status")
            print("- q: Quit")
            return True
            
        if command.lower() == 'status':
            status = "READY" if self.navigator_ready else "INITIALIZING"
            print(f"Status: {status}")
            print(f"Location: {self.current_destination or 'Home'}")
            if self.canceled_tables:
                print(f"Canceled Tables: {self.canceled_tables}")
            return True
        
        # Direct home command
        if command.lower() in ['home', 'h']:
            self.go_home()
            return True
            
        # Handle cl command to go home
        if command.lower() == 'cl':
            print('Canceling current task and going home')
            if self.navigator_ready:
                self.nav.cancelTask()
                time.sleep(0.5)
                self.go_home()
            return True
            
        # Table cancellation commands
        if command.lower().startswith('c') and len(command) == 2 and command[1].isdigit():
            table_num = int(command[1])
            if 1 <= table_num <= 3:
                self.canceled_tables.append(table_num)
                print(f'Table {table_num} order canceled!')
            return True
            
        # Single table orders
        if command.lower().startswith('t') and len(command) >= 2 and command[1:].isdigit():
            try:
                table_num = int(command[1:])
                if 1 <= table_num <= 3:
                    self.handle_orders([table_num])
                return True
            except ValueError:
                print('Invalid table format. Use t1, t2, or t3')
                return True
                
        # Multiple table orders
        if ',' in command or ' ' in command:
            separator = ',' if ',' in command else ' '
            try:
                table_nums = []
                for item in command.split(separator):
                    item = item.strip()
                    if item.startswith('t') and item[1:].isdigit():
                        num = int(item[1:])
                    elif item.isdigit():
                        num = int(item)
                    else:
                        continue
                        
                    if 1 <= num <= 3:
                        table_nums.append(num)
                
                if table_nums:
                    self.handle_orders(table_nums)
                return True
            except ValueError:
                print('Invalid format for multiple tables')
                return True
                
        print(f'Unknown command: {command}')
        print('Type "help" for commands')
        return True
        
    def run(self):
        """Main robot operation loop"""
        print("\nCOMMANDS: t1/t2/t3 (single table), 1,2,3 (multiple tables)")
        print("cl (cancel & go home), c1/c2/c3 (cancel table), help, q (quit)")
        
        running = True
        while running and rclpy.ok():
            try:
                # Check for immediate input
                user_input = InputHandler.check_input(0.1)
                if user_input:
                    # Process cancellations immediately
                    if user_input.lower() in ['c1', 'c2', 'c3']:
                        table_num = int(user_input.lower()[1])
                        self.canceled_tables.append(table_num)
                        print(f'Table {table_num} order canceled!')
                        continue
                    elif user_input.lower() == 'cl' and self.navigator_ready:
                        self.nav.cancelTask()
                        print('Current task canceled! Going home...')
                        self.go_home()
                        continue
                    elif user_input.lower() == 'q':
                        running = False
                        continue
                        
                    # Process other commands
                    running = self.process_command(user_input)
                    continue
                    
                # Get command from user when no immediate input
                command = input("\nEnter command: ")
                running = self.process_command(command)
                
            except KeyboardInterrupt:
                running = False
                
        # Clean shutdown
        if self.navigator_ready:
            try:
                self.nav.lifecycleShutdown()
            except Exception as e:
                self.get_logger().error(f'Error during shutdown: {e}')

def main():
    try:
        rclpy.init()
        robot = RestaurantRobot()
        robot.run()
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
