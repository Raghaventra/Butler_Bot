#!/usr/bin/env python3

# Imports - standard ROS stuff plus what we need for map clicking
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped
import threading
import time
import signal
import re
import math

# Initial locations - we'll replace these with clicks, but keep as fallbacks
DEFAULT_LOCATIONS = {
    "home":    (0.0, 0.0, 0.0),
    "kitchen": (2.0, 0.0, 0.0),
    "table1":  (4.0, 1.0, 0.0),
    "table2":  (4.0, -1.0, 0.0),
    "table3":  (5.0, 0.0, 0.0)
}

# Our actual locations will be stored here once clicked
LOCATIONS = {}

# Timeout for user input - 10 seconds feels about right
USER_TIMEOUT = 10

# Flag to track if we're currently in location selection mode
selecting_location = False
current_location_name = None

# Custom exception for our timeout mechanism
class TimeoutException(Exception):
    pass

# Signal handler for timeouts
def alarm_handler(signum, frame):
    raise TimeoutException

# Callback for when someone clicks on the map
def map_click_callback(clicked_point):
    global selecting_location, current_location_name, LOCATIONS
    
    if not selecting_location:
        return
        
    # Store the clicked point
    x = clicked_point.point.x
    y = clicked_point.point.y
    
    # Let's assume the robot should face forward (yaw=0)
    # Could extend this to calculate orientation based on context
    yaw = 0.0
    
    # Save location and print confirmation
    LOCATIONS[current_location_name] = (x, y, yaw)
    rospy.loginfo(f"✓ Saved location '{current_location_name}' at x={x:.2f}, y={y:.2f}")
    
    # We're done selecting this location
    selecting_location = False

# User input function with timeout
def get_user_input(prompt, timeout=USER_TIMEOUT):
    """
    Gets user input but doesn't hang forever - returns None if user is AFK
    """
    signal.signal(signal.SIGALRM, alarm_handler)
    signal.alarm(timeout)
    
    try:
        # Get user input
        answer = input(prompt)
        signal.alarm(0)  # Turn off the alarm
        
        # If they just hit enter, treat as "yes"
        if answer.strip() == "":
            return "y"
            
        return answer.strip().lower()
        
    except TimeoutException:
        print("\n⏰ Timeout reached! The robot is moving on...")
        return None

# Navigation function
def navigate_to(location_name):
    """
    Sends the robot to the specified location using move_base
    Returns True if successful, False otherwise
    """
    # Make sure we have this location
    if location_name not in LOCATIONS:
        rospy.logerr(f"🚫 Can't find location '{location_name}' in my database!")
        return False

    # Connect to the move_base action server
    nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    # Wait for the server to be available (max 5 seconds)
    server_available = nav_client.wait_for_server(timeout=rospy.Duration(5.0))
    if not server_available:
        rospy.logerr("❌ Timed out waiting for move_base server! Is navigation running?")
        return False
        
    # Get location coordinates and orientation
    x, y, yaw = LOCATIONS[location_name]
    
    # Convert yaw angle to quaternion
    quat = quaternion_from_euler(0, 0, yaw)

    # Create navigation goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    # Send the robot on its way!
    rospy.loginfo(f"🚶 Taking off to {location_name} at coordinates ({x:.2f}, {y:.2f})...")
    nav_client.send_goal(goal)
    
    # Wait for the result
    nav_client.wait_for_result()
    result = nav_client.get_state()

    # Check if we got there
    if result == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"🎯 Successfully reached {location_name}!")
        return True
    else:
        rospy.logwarn(f"⚠️ Couldn't reach {location_name}. Navigation failed with code {result}.")
        return False

# Single order handler
def handle_single_order(table_num):
    """
    Process a delivery to a single table:
    1. Go to kitchen to pick up the order
    2. Deliver to the table
    3. Return home
    """
    table_name = f"table{table_num}"
    rospy.loginfo(f"🍽️ Starting delivery to {table_name}...")

    # First, head to the kitchen
    if not navigate_to("kitchen"):
        rospy.logerr("❌ Failed to reach the kitchen. Aborting this order.")
        return

    # We're at the kitchen - wait for staff to confirm order is ready
    kitchen_response = get_user_input("🍳 At kitchen: Is the order ready? (y/n/c to cancel): ")
    
    # Handle various kitchen responses
    if kitchen_response is None or kitchen_response == "n":
        rospy.loginfo("⏪ Order not ready yet. Heading back home.")
        navigate_to("home")
        return
    elif kitchen_response == "c":
        rospy.loginfo("❌ Order canceled at kitchen. Returning to home position.")
        navigate_to("home")
        return

    # Order confirmed, let's head to the table
    if not navigate_to(table_name):
        rospy.logerr(f"❌ Navigation to {table_name} failed. Returning to kitchen.")
        navigate_to("kitchen")
        navigate_to("home")
        return

    # We made it to the table! Wait for customer to take their order
    table_response = get_user_input(f"🍽️ At {table_name}: Has customer received their order? (y/n/c to cancel): ")
    
    # Handle the table responses
    if table_response is None or table_response == "n":
        rospy.loginfo(f"⏪ Customer not ready at {table_name}. Taking food back to kitchen.")
        navigate_to("kitchen")
        navigate_to("home")
        return
    elif table_response == "c":
        rospy.loginfo(f"❌ Delivery canceled at {table_name}. Returning to kitchen.")
        navigate_to("kitchen")
        navigate_to("home")
        return

    # Successful delivery! 
    rospy.loginfo(f"✅ Order successfully delivered to {table_name}! Heading home.")
    navigate_to("home")

# Multiple order handler
def handle_multiple_orders(tables):
    """
    Efficient multi-table delivery:
    1. Pickup all orders from kitchen
    2. Deliver to each table in sequence
    3. Return to kitchen then home
    """
    rospy.loginfo(f"🛒 Starting multi-table delivery to tables: {', '.join(tables)}")
    
    # First stop: kitchen to get all the orders
    if not navigate_to("kitchen"):
        rospy.logerr("❌ Couldn't reach kitchen. Canceling multi-table delivery.")
        return

    # Track which tables we successfully delivered to
    successful_deliveries = []
    
    # Visit each table in sequence
    for table_num in tables:
        table_name = f"table{table_num}"
        rospy.loginfo(f"🚗 Now delivering to {table_name}...")
        
        # Try to get to this table
        if not navigate_to(table_name):
            rospy.logwarn(f"⚠️ Couldn't reach {table_name}. Skipping this table.")
            continue

        # At the table - get confirmation
        response = get_user_input(f"🍽️ At {table_name}: Confirm delivery? (y/n/c to cancel): ")
        
        # Process the response
        if response is None or response == "n":
            rospy.loginfo(f"⏭️ No confirmation at {table_name}. Continuing to next table.")
        elif response == "c":
            rospy.loginfo(f"❌ Order for {table_name} canceled. Moving on.")
        else:
            rospy.loginfo(f"✅ Successfully delivered to {table_name}!")
            successful_deliveries.append(table_name)

    # Done with all deliveries
    if successful_deliveries:
        rospy.loginfo(f"🎉 Completed deliveries to: {', '.join(successful_deliveries)}")
    else:
        rospy.loginfo("😕 No successful deliveries in this run.")
        
    # Return to kitchen then home
    rospy.loginfo("🏁 Finishing delivery run, heading back to kitchen.")
    navigate_to("kitchen")
    navigate_to("home")

# Set up locations via map clicks
def setup_locations():
    """
    Interactive setup where user clicks to define locations
    """
    global selecting_location, current_location_name, LOCATIONS
    
    # First, copy our defaults as a fallback
    LOCATIONS = DEFAULT_LOCATIONS.copy()
    
    print("\n--- BUTLER ROBOT LOCATION SETUP ---")
    print("Click on the RViz map to set each location.")
    print("(Press Enter to keep default coordinates)\n")
    
    # For each location, prompt user to click
    for loc_name in ["home", "kitchen", "table1", "table2", "table3"]:
        default_x, default_y, default_yaw = DEFAULT_LOCATIONS[loc_name]
        
        response = input(f"Set location for '{loc_name}' (current: {default_x:.1f}, {default_y:.1f})? (y/n): ")
        
        if response.lower() == 'y':
            # Set up for click selection
            current_location_name = loc_name
            selecting_location = True
            print(f"👆 Please click on the map to set location for '{loc_name}'...")
            
            # Wait until location is selected (or timeout)
            start_time = time.time()
            while selecting_location and time.time() - start_time < 20:
                time.sleep(0.1)
                
            if selecting_location:
                # We timed out
                selecting_location = False
                print(f"⏰ Timed out waiting for click. Using default for '{loc_name}'.")
        else:
            print(f"✓ Using default position for '{loc_name}'")
    
    print("\n✅ Location setup complete!")
    
def main():
    # Initialize our ROS node
    rospy.init_node("butler_robot")
    rospy.loginfo("🤖 Butler Robot Service - Starting Up!")
    
    # Set up our click subscriber
    rospy.Subscriber("/clicked_point", PointStamped, map_click_callback)
    
    # Run the location setup
    setup_locations()
    
    # Main loop - keep running until shutdown
    while not rospy.is_shutdown():
        print("\n" + "=" * 40)
        print("🤖 BUTLER ROBOT - MAIN MENU")
        print("=" * 40)
        print("Select an option:")
        print("  s - Single table delivery")
        print("  m - Multi-table delivery")
        print("  r - Reconfigure locations")
        print("  q - Quit")
        
        choice = input("\nYour choice: ").strip().lower()
        
        if choice == 'q':
            break
            
        elif choice == 's':
            # Get table number for single delivery
            table = input("Which table? (1, 2, or 3): ").strip()
            if table not in ["1", "2", "3"]:
                rospy.logwarn("⚠️ Invalid table number. Please enter 1, 2, or 3.")
                continue
                
            handle_single_order(table)
            
        elif choice == 'm':
            # Get list of tables for multi-delivery
            tables_input = input("Enter table numbers (e.g., 1,2,3 or 1 2 3): ")
            
            # Parse into a list (accepting spaces or commas)
            table_list = re.split(r'[\s,]+', tables_input.strip())
            
            # Keep only valid table numbers
            valid_tables = [t for t in table_list if t in ["1", "2", "3"]]
            
            if not valid_tables:
                rospy.logwarn("⚠️ No valid table numbers entered.")
                continue
                
            # Go deliver to these tables
            handle_multiple_orders(valid_tables)
            
        elif choice == 'r':
            # Reconfigure locations
            setup_locations()
            
        else:
            rospy.logwarn("⚠️ Unknown option. Please try again.")
    
    rospy.loginfo("👋 Butler Robot shutting down. Have a nice day!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by ROS. Shutting down.")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received. Shutting down.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {str(e)}")