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
            ('nav_timeout', 100.0),
            ('nav2_init_timeout', 60.0),
            ('skip_nav2_check', True)
        ])
        
        self.confirmation_timeout = self.get_parameter('confirmation_timeout').value
        self.wait_time_between_orders = self.get_parameter('wait_time_between_orders').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.nav2_init_timeout = self.get_parameter('nav2_init_timeout').value
        self.skip_nav2_check = self.get_parameter('skip_nav2_check').value
        
        self.locations = {
            'HOME': {"x": 0.0, "y": 0.0, "theta": 0.0},
            'KITCHEN': {"x": 1.5, "y": -0.5, "theta": 0.11},
            'TABLE1': {"x": 4.6, "y": 1.3, "theta": 0.11},
            'TABLE2': {"x": 4.6, "y": 0.0, "theta": 0.11},
            'TABLE3': {"x": 4.6, "y": -1.3, "theta": 0.11}
        }
        
        self.confirmation_received = False
        self.canceled_tables = []
        self.current_destination = None
        self.navigator_ready = True  
        self.nav_init_error = None
        self.nav = None
        
        try:
            self._initialize_navigation()
            print('\nNavigation ready! Send robot with "t1" or "1,2,3"')
        except Exception as e:
            self.nav_init_error = f"Navigation initialization error: {str(e)}"
            print(f"Navigation initialization error: {str(e)}")
        
    def _initialize_navigation(self):
        try:
            self.nav = BasicNavigator()
            self.quaternions = [quaternion_from_euler(0.0, 0.0, angle) 
                               for angle in [0.0, 0.0, 0.0, 0.0]]
                
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
            
                       
        except Exception as e:
            raise
    
    def _create_goal_pose(self, position):
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
    
    def go_home(self):
        
        try:
            if not self.nav.isTaskComplete():
                self.nav.cancelTask()
                time.sleep(0.5)
        except Exception as e:
            pass
        
        result = self.navigate_to('HOME', "Home", interactive=False)
        return result
        
    def navigate_to(self, location_key, location_name, interactive=True):
        
        target_table_num = None
        if "TABLE" in location_key:
            try:
                target_table_num = int(location_key.replace("TABLE", ""))
            except:
                pass
                
        if interactive:
            print('Commands: "cl" to cancel and go home, "c1/c2/c3" to cancel specific table')
            
        self.current_destination = location_name
        
        if location_key not in self.locations:
            return TaskResult.FAILED
            
        goal_pose = self._create_goal_pose(self.locations[location_key])
        
        try:
            if not self.nav.isTaskComplete():
                self.nav.cancelTask()
                time.sleep(0.5)
        except Exception as e:
            pass
            
        try:
            self.nav.goToPose(goal_pose)
        except Exception as e:
            return TaskResult.FAILED
        
        last_status_time = time.time()
        start_time = time.time()
        
        print(f"Distance to {location_name}: calculating...")
        
        while not self.nav.isTaskComplete():
            if not interactive and time.time() - start_time > self.nav_timeout:
                self.nav.cancelTask()
                return TaskResult.FAILED
                
            if interactive:
                user_input = InputHandler.check_input(0.1)
                if user_input:
                    if user_input.lower() in ['c1', 'c2', 'c3']:
                        canceled_table = int(user_input.lower()[1])
                        self.canceled_tables.append(canceled_table)
                        
                        if target_table_num and target_table_num == canceled_table:
                            self.nav.cancelTask()
                            return TaskResult.CANCELED
                    
                    elif user_input.lower() == 'cl':
                        self.nav.cancelTask()
                        time.sleep(0.5)
                        self.go_home()
                        return TaskResult.CANCELED
            
            current_time = time.time()
            if current_time - last_status_time >= 1.0:
                last_status_time = current_time
                try:
                    feedback = self.nav.getFeedback()
                    if feedback and hasattr(feedback, 'distance_remaining'):
                        print(f"Distance to {location_name}: {feedback.distance_remaining:.2f}m")
                except Exception as e:
                    pass
            
            time.sleep(0.05)
        
        try:
            result = self.nav.getResult()
        except Exception as e:
            result = TaskResult.FAILED
            
        return result
    
    def wait_for_confirmation(self, location):
        table_num = None
        if "TABLE" in location.upper():
            try:
                table_num = int(location.upper().replace("TABLE", "").strip())
            except:
                pass
        
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
                    self.go_home()
                    return False
                elif table_num and user_input.lower() == f'c{table_num}':
                    self.canceled_tables.append(table_num)
                    return False
        return False
    
    def handle_orders(self, table_nums):
        
        if not isinstance(table_nums, list):
            table_nums = [table_nums]
            
        valid_orders = [t for t in table_nums 
                        if f'TABLE{t}' in self.locations and t not in self.canceled_tables]
                        
        if not valid_orders:
            return False
            
        result = self.navigate_to('KITCHEN', "Kitchen")
        if result != TaskResult.SUCCEEDED:
            return False
            
        valid_orders = [t for t in valid_orders if t not in self.canceled_tables]
        if not valid_orders:
            self.navigate_to('HOME', "Home", interactive=False)
            return False
            
        if not self.wait_for_confirmation("Kitchen"):
            return False
                
        valid_orders = [t for t in valid_orders if t not in self.canceled_tables]
        if not valid_orders:
            self.navigate_to('HOME', "Home", interactive=False)
            return False
            
        successful_tables = []
        failed_tables = []
        
        for table_num in valid_orders:
            if table_num in self.canceled_tables:
                continue
                
            table_key = f'TABLE{table_num}'
            result = self.navigate_to(table_key, f"Table {table_num}")
            
            if result != TaskResult.SUCCEEDED:
                failed_tables.append(table_num)
                continue
                
            if table_num in self.canceled_tables:
                continue
                
            if self.wait_for_confirmation(f"Table {table_num}"):
                successful_tables.append(table_num)
            else:
                if table_num not in self.canceled_tables:
                    failed_tables.append(table_num)
                    
        if failed_tables or self.canceled_tables:
            self.navigate_to('KITCHEN', "Kitchen", interactive=False)
            
        self.navigate_to('HOME', "Home", interactive=False)
        
        print('\n== Delivery Summary ==')
        if successful_tables:
            print(f'Successfully delivered to Tables: {successful_tables}')
        if self.canceled_tables:
            print(f'Canceled deliveries for Tables: {self.canceled_tables}')
        if failed_tables:
            print(f'No confirmation from Tables: {failed_tables}')
            
        self.canceled_tables = []
        
        return len(successful_tables) > 0
    
    def process_command(self, command):
        if not command:
            return True
            
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
            print("- debug: Show navigation debug info")
            print("- test: Run a simple navigation test")
            print("- q: Quit")
            return True
            
        if command.lower() == 'status':
            status = "READY" if self.navigator_ready else "INITIALIZING"
            if self.nav_init_error:
                status = f"ERROR: {self.nav_init_error}"
            print(f"Status: {status}")
            print(f"Location: {self.current_destination or 'Home'}")
            if self.canceled_tables:
                print(f"Canceled Tables: {self.canceled_tables}")
            return True
            
        if command.lower() == 'debug':
            print("\n== Debug Information ==")
            print(f"Navigation Ready: {self.navigator_ready}")
            print(f"Navigation Init Error: {self.nav_init_error or 'None'}")
            print(f"Skip Nav2 Check: {self.skip_nav2_check}")
            print(f"Known Locations: {list(self.locations.keys())}")
            if self.navigator_ready:
                try:
                    print(f"Nav Task Complete: {self.nav.isTaskComplete()}")
                except Exception as e:
                    pass
            return True
            
        if command.lower() == 'test':
            print("Running simple navigation test...")
            self.navigate_to('HOME', "Home", interactive=False)
            print("Navigation test complete.")
            return True
        
        if command.lower() in ['home', 'h']:
            self.go_home()
            return True
            
        if command.lower() == 'cl':
            try:
                self.nav.cancelTask()
                time.sleep(0.5)
                self.go_home()
            except Exception as e:
                pass
            return True
            
        if command.lower().startswith('c') and len(command) == 2 and command[1].isdigit():
            table_num = int(command[1])
            if 1 <= table_num <= 3:
                self.canceled_tables.append(table_num)
                print(f'Table {table_num} order canceled!')
            return True
            
        if command.lower().startswith('t') and len(command) >= 2 and command[1:].isdigit():
            try:
                table_num = int(command[1:])
                if 1 <= table_num <= 3:
                    self.handle_orders([table_num])
                else:
                    print(f'Invalid table number: {table_num}. Use tables 1-3.')
                return True
            except ValueError:
                print('Invalid table format. Use t1, t2, or t3')
                return True
                
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
                else:
                    print('No valid table numbers found')
                return True
            except ValueError:
                print('Invalid format for multiple tables')
                return True
                
        print(f'Unknown command: {command}')
        print('Type "help" for commands')
        return True
        
    def run(self):
        print("\nCOMMANDS: t1/t2/t3 (single table), 1,2,3 (multiple tables)")
        print("cl (cancel & go home), c1/c2/c3 (cancel table), help, q (quit)")
        
        running = True
        while running and rclpy.ok():
            try:
                user_input = InputHandler.check_input(0.1)
                if user_input:
                    if user_input.lower() in ['c1', 'c2', 'c3']:
                        table_num = int(user_input.lower()[1])
                        self.canceled_tables.append(table_num)
                        print(f'Table {table_num} order canceled!')
                        continue
                    elif user_input.lower() == 'cl':
                        try:
                            self.nav.cancelTask()
                            print('Current task canceled! Going home...')
                            self.go_home()
                        except Exception as e:
                            pass
                        continue
                    elif user_input.lower() == 'q':
                        running = False
                        continue
                        
                    running = self.process_command(user_input)
                    continue
                    
                command = input("\nEnter command: ")
                running = self.process_command(command)
                
            except KeyboardInterrupt:
                running = False
            except Exception as e:
                pass
                
        if self.navigator_ready:
            try:
                self.nav.lifecycleShutdown()
            except Exception as e:
                pass

def main():
    try:
        rclpy.init()
        robot = RestaurantRobot()
        robot.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
