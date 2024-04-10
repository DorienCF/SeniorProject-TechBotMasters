import os  # For terminal clearing (optional)
import json  # For Database CRUD.
import datetime # For the time based parts
import subprocess  # for opening new terminal
import time


DATABASE_FILE = "patient_database.json"
LOADED_ROUTE = "Route not loaded"
## Clears the terminal screen of whatever was previously on there
def clear_screen():
    # Clear the terminal screen for a fresh look (optional)
    if os.name == 'posix':  # POSIX systems (Linux, macOS)
        os.system('clear')
    else:
        os.system('cls')  # Windows systems

def display_menu():
    clear_screen()  # Clear the screen before displaying the menu (optional)

    print("   _______                   ________    ")
    print("  |ooooooo|      ____       | __  __ |   ")
    print("  |[]+++[]|     [____]      |/  \/  \|   ")
    print("  |+ ___ +|     ]()()[      |\__/\__/|   ")
    print("  |:|   |:|   ___\__/___    |[][][][]|   ")
    print("  |:|___|:|  |__|    |__|   |++++++++|   ")
    print("  |[]===[]|   |_|_/\_|_|    | ______ |   ")
    print("_ ||||||||| _ | | __ | | __ ||______|| __|")
    print("  |_______|   |_|[::]|_|    |________|   \ ")
    print("              \_|_||_|_/               jro\ ")
    print("                |_||_|                     \ ")
    print("               _|_||_|_                     \ ")
    print("#################################################")




    print("# __| |______________________________________________________________________________| |__")  
    print("# __   ______________________________________________________________________________   __")  
    print("#   | |                                                                              | |  ")  
    print("#   | | _   _                                   __  __            _     _            | |  ")  
    print("#   | || | |_   _ _ __ ___   __ _ _ __         |  \/  | __ _  ___| |__ (_)_ __   ___ | |  ")  
    print("#   | || |_| | | | | '_ ` _ \ / _` | '_ \ _____| |\/| |/ _` |/ __| '_ \| | '_ \ / _ \| |  ")  
    print("#   | ||  _  | |_| | | | | | | (_| | | | |_____| |  | | (_| | (__| | | | | | | |  __/| |  ")  
    print("#   | ||_| |_|\__,_|_| |_| |_|\__,_|_| |_|     |_|  |_|\__,_|\___|_| |_|_|_| |_|\___|| |  ")  
    print("#   | |                                                                              | |  ")  
    print("#   | | ___       _             __                                                   | |  ")  
    print("#   | ||_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___                                    | |  ")  
    print("#   | | | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \                                   | |  ")  
    print("#   | | | || | | | ||  __/ |  |  _| (_| | (_|  __/                                   | |  ")  
    print("#   | ||___|_| |_|\__\___|_|  |_|  \__,_|\___\___|                                   | |  ")  
    print("#   | |                                                                              | |  ")  
    print("#   | |__        __   _                                                              | |  ")  
    print("#   | |\ \      / /__| | ___ ___  _ __ ___   ___                                     | |  ")  
    print("#   | | \ \ /\ / / _ \ |/ __/ _ \| '_ ` _ \ / _ \                                    | |  ")  
    print("#   | |  \ V  V /  __/ | (_| (_) | | | | | |  __/                                    | |  ")  
    print("#   | |   \_/\_/ \___|_|\___\___/|_| |_| |_|\___|                                    | |  ")  
    print("# __| |______________________________________________________________________________| |__")  
    print("# __   ______________________________________________________________________________   __")  
    print("#   | |                                                                              | |  ")
    #print(f'\n\n\n Current route: {LOADED_ROUTE}')
    print("\t (1) Plan a Route ")  # Clearer explanation
    print("\t\t- Create a route to optimize medicine delivery,")
    print("\t\t  ensuring patient safety.")
    print("\t (2) Load Route")
    print("\t\t- Load a previously saved route for execution.")
    print("\t (3) View Route History")
    print("\t\t- Review past routes for reference.")
    print("\t (4) Execute Route")
    print("\t\t- Start the robot's journey based on the loaded route.")
    print("\t (5) Edit Patient Data")  # New option
    print("\t\t- Modify patient information (e.g., names, locations, medication)")
    print("\t (6) Exit")

def route_loading():
  clear_screen()
  print("Route Loading Functionality\n")
  filename = input("Enter the name of the route file (without extension): ")

  # Check if the file exists in the current directory and has a .txt extension
  if os.path.exists(f"{filename}.txt"):
      with open(f"{filename}.txt", 'r') as file:
          route_data = file.readlines()
          print(f"\nRoute loaded successfully from {filename}.txt:")
          for line in route_data:
              print(line.strip())
              LOADED_ROUTE = route_data
              return LOADED_ROUTE

      # Prompt the user for further action
      while True:
          choice = input("\nDo you want to change the file or go back to the main menu? (change/menu): ")
          if choice.lower() == "change":
              filename = input("Enter the name of the new route file (without extension): ")
              if os.path.exists(f"{filename}.txt"):
                  with open(f"{filename}.txt", 'r') as new_file:
                      route_data = new_file.readlines()
                      print(f"\nRoute loaded successfully from {filename}.txt:")
                      for line in route_data:
                          print(line.strip())
                          LOADED_ROUTE = route_data
                          return LOADED_ROUTE
              else:
                  print(f"Error: File {filename}.txt not found in the current directory.")
          elif choice.lower() == "menu":
              print("Returning to the main menu.")
              # Save the route data or filename as a main variable here if needed
              break
          else:
              print("Invalid choice. Please enter 'change' or 'menu'.")
  else:
      print(f"Error: File {filename}.txt not found in the current directory.")

def load_database():
  try:
      with open(DATABASE_FILE, 'r') as file:
          return json.load(file)
  except FileNotFoundError:
      return {'patients': {}, 'medications': {}, 'prescribers': {}}

def save_database(database):
  with open(DATABASE_FILE, 'w') as file:
      json.dump(database, file, indent=4)
## Sources the database to print our the patient names and their information
def display_patients(database):
  print("\nPatients Database:")
  for patient_id, patient_info in database['patients'].items():
      print(f"Patient ID: {patient_id}")
      print(f"Name: {patient_info['name']}")
      print(f"Room: {patient_info['room']}")
      print(f"Coordinates: {patient_info['coordinates']}")
      print("Medications:")
      for medication_id, medication_info in patient_info['medications'].items():
          print(f"\tMedication ID: {medication_id}")
          print(f"\tMedication Name: {medication_info['name']}")
          print(f"\tDosage: {medication_info['dosage']}")
          print(f"\tDatetime: {medication_info['datetime']}")
          print(f"\tPrescriber ID: {medication_info['prescriber_id']}")
          print()
      print()
## Adds a new patient to the database. 
def add_patient(database):
  name = input("Enter patient name (Last First): ")
  room = input("Enter patient room: ")
  coordinates = input("Enter patient coordinates (x,y): ").split(',')
  patient_id = input("Enter patient ID: ")
  database['patients'][patient_id] = {
      'name': name,
      'room': room,
      'coordinates': [int(coord) for coord in coordinates],
      'medications': {}
  }
  print("Patient added successfully.")
# Updates the patient's information to the database.
def update_patient(database):
  patient_id = input("Enter patient ID to update: ")
  if patient_id in database['patients']:
      name = input("Enter updated patient name (Last First): ")
      database['patients'][patient_id]['name'] = name
      print("Patient information updated successfully.")
  else:
      print("Patient ID not found.")
# Deletes a patient from the database.
def delete_patient(database):
  patient_id = input("Enter patient ID to delete: ")
  if patient_id in database['patients']:
      del database['patients'][patient_id]
      print("Patient deleted successfully.")
  else:
      print("Patient ID not found.")
# Edits patient information to the database.
def edit_patient_data(database):
  while True:
      print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
      print("#############################################################")
      print("\nEdit Patient Data:")
      print("1. View Patients")
      print("2. Add Patient")
      print("3. Update Patient")
      print("4. Delete Patient")
      print("5. Go back to the main menu")

      choice = input("Enter your choice (1-5): ")
      if choice == '1':
          display_patients(database)
      elif choice == '2':
          add_patient(database)
      elif choice == '3':
          update_patient(database)
      elif choice == '4':
          delete_patient(database)
      elif choice == '5':
          print("Returning to the main menu.")
          break
      else:
          print("Invalid choice. Please enter a number between 1 and 5.")

def option_5(database):
  edit_patient_data(database)
# Execute the functionaly for planning the route.
def route_planning(database):
  print("Route planning functionality in progress...")
  # Retrieve patient data from the database
  patients = database['patients']
  
  # Display available patients for selection
  print("Available Patients:")
  for patient_id, patient_info in patients.items():
      print(f"ID: {patient_id}, Name: {patient_info['name']}, Room: {patient_info['room']}")
  
  # Initialize route
  route = []
  
  # Request employee ID for verification
  employee_id = input("Enter Your Employee ID: ")
  
  # Input patient IDs and medicines for each patient
  num_patients = int(input("Enter the number of patients you plan on treating: "))
  patient_ids = []
  medicine_ids = []
  for i in range(num_patients):
      patient_id = input(f"Enter Patient {i+1} ID: ")
      if patient_id not in patients:
          print("Invalid patient ID.")
          return
      patient_ids.append(patient_id)
  
      # Check if the patient has medications
      if patients[patient_id]['medications']:
          medication_id = input(f"Enter Medicine ID for Patient {i+1}: ")
          if medication_id not in patients[patient_id]['medications']:
              print("Invalid medicine ID.")
              return
          medicine_ids.append(medication_id)
      else:
          print("Patient has no medications.")
          return
  
  # Confirm route selection
  print("\nConfirm Route:")
  print(f"Employee: {employee_id}")
  print(f"Number of Patients: {num_patients}")
  for i in range(num_patients):
      print(f"\tPatient {i+1}: {patient_ids[i]} - Medicine: {medicine_ids[i]}")
  
  confirmation = input("\nIs this route correct? (y/n): ")
  if confirmation.lower() == 'y':
      # Build route
      for patient_id in patient_ids:
          route.append(database['patients'][patient_id]['coordinates'])
  
      # Add return to base for resupply
      route.insert(0, (0, 0))
      route.append((0, 0))
  
      # Save route to file
      filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_route.txt"
      save_route(route, filename)
      print(f"Route saved successfully as {filename}")
  else:
      print("Route planning cancelled.")
# Saves the route to a file.
def save_route(route, filename):
  with open(filename, 'w') as file:
      file.write("Route:\n")
      for point in route:
          file.write(f"\t- {point}\n")
# Loads the patient information database
def load_database():
  try:
      with open(DATABASE_FILE, 'r') as file:
          return json.load(file)
  except FileNotFoundError:
      return {'patients': {}, 'medications': {}, 'prescribers': {}}
# Saves the patient information database
def save_database(database):
  with open(DATABASE_FILE, 'w') as file:
      json.dump(database, file, indent=4)
# Runs the funationaly to see what routes have been previously run. 
def route_history_viewing():
  clear_screen()
  print("Route History Viewing Functionality\n")
  try:
      with open("route_history.txt", 'r') as file:
          history_data = file.read()
          print("Route History:")
          print(history_data)
  except FileNotFoundError:
      print("No route history found.")

  while True:
      choice = input("\nEnter 'back' to return to the main menu: ")
      if choice.lower() == 'back':
          print("Returning to the main menu.")
          break
      else:
          print("Invalid choice. Please enter 'back' to return to the main menu.")


def display_patients(database):
  print("\nPatients Database:")
  for patient_id, patient_info in database['patients'].items():
      print(f"Patient ID: {patient_id}")
      print(f"Name: {patient_info['name']}")
      print(f"Room: {patient_info['room']}")
      print(f"Coordinates: {patient_info['coordinates']}")
      print("Medications:")
      for medication_id, medication_info in patient_info['medications'].items():
          print(f"\tMedication ID: {medication_id}")
          print(f"\tMedication Name: {medication_info['name']}")
          print(f"\tDosage: {medication_info['dosage']}")
          print(f"\tDatetime: {medication_info['datetime']}")
          print(f"\tPrescriber ID: {medication_info['prescriber_id']}")
          print()
      print("\n")
    
def update_patient(database):
  patient_id = input("Enter patient ID to update: ")
  if patient_id in database['patients']:
      name = input("Enter updated patient name (Last First): ")
      database['patients'][patient_id]['name'] = name
      print("Patient information updated successfully.")
  else:
      print("Patient ID not found.")
# deletes patient data from the database.
def delete_patient(database):
  patient_id = input("Enter patient ID to delete: ")
  if patient_id in database['patients']:
      del database['patients'][patient_id]
      print("Patient deleted successfully.")
  else:
      print("Patient ID not found.")
## Runs the route either by teleopration or autonomously. 
def execute_route(database, LOADED_ROUTE):
    # Request valid Prescriber ID
    prescriber_id = input("Enter Prescriber ID: ")
    # Validate Prescriber ID against the database
    if not validate_prescriber_id(prescriber_id, database):
        print("Invalid Prescriber ID. Route execution aborted.")
        return

    # Run ROS navigation program based on user's choice
    print("Executing route functionality in progress...")
    print("ROS Navigation Program is running...")
    choice = input("Do you want to run the plan autonomously (A) or via teleoperation (T)? (A/T): ").upper()
    if choice == 'A':
        print("Launching RVIZ2 for autonomous operation...")
        # Launch RVIZ2 for autonomous operation in a separate terminal
        try:
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "ros2 launch turtlebot4_viz view_robot.launch.py; exec $SHELL"])
            time.sleep(2)  # Delay to ensure RVIZ2 is fully launched before running ROS code
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "ros2 run turtlebot4_python_tutorials nav_through_poses; exec $SHELL"])
        except Exception as e:
            print("Error launching RVIZ2 or ROS code:", e)
            return

    elif choice == 'T':
        print("Launching RVIZ2 for teleoperation...")
        # Launch RVIZ2 for teleoperation in a separate terminal
        try:
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "ros2 launch turtlebot4_viz view_robot.launch.py; exec $SHELL"])
            time.sleep(2)  # Delay to ensure RVIZ2 is fully launched before running ROS code
            subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec $SHELL"])
        except Exception as e:
            print("Error launching RVIZ2 or ROS code:", e)
            return

    else:
        print("Invalid choice. Route execution aborted.")
        return

    # Append the executed route to the log file
    with open("route_log.txt", 'a') as logfile:
        logfile.write("Executed Route:\n")
        # Write details of the executed route to the log file


def edit_patient_data(database):
  while True:
      print("\nEdit Patient Data:")
      print("1. View Patients")
      print("2. Add Patient")
      print("3. Update Patient")
      print("4. Delete Patient")
      print("5. Go back to the main menu")

      choice = input("Enter your choice (1-5): ")
      if choice == '1':
          display_patients(database)
      elif choice == '2':
          add_patient(database)
      elif choice == '3':
          update_patient(database)
      elif choice == '4':
          delete_patient(database)
      elif choice == '5':
          print("Returning to the main menu.")
          break
      else:
          print("Invalid choice. Please enter a number between 1 and 5.")
    
  # Load database at the beginning of the main function
  database = load_database()

def option_5(database):
  edit_patient_data(database)

def route_planning(database):
  print("Route planning functionality in progress...")
  # Retrieve patient data from the database
  patients = database['patients']
  
  # Display available patients for selection
  print("Available Patients:")
  for patient_id, patient_info in patients.items():
      print(f"ID: {patient_id}, Name: {patient_info['name']}, Room: {patient_info['room']}")
  
  # Initialize route
  route = []
  
  # Request employee ID for verification
  employee_id = input("Enter Your Employee ID: ")
  
  # Input patient IDs and medicines for each patient
  num_patients = int(input("Enter the number of patients you plan on treating: "))
  patient_ids = []
  medicine_ids = []
  for i in range(num_patients):
      patient_id = input(f"Enter Patient {i+1} ID: ")
      if patient_id not in patients:
          print("Invalid patient ID.")
          return
      patient_ids.append(patient_id)
  
      # Check if the patient has medications
      if patients[patient_id]['medications']:
          medication_id = input(f"Enter Medicine ID for Patient {i+1}: ")
          if medication_id not in patients[patient_id]['medications']:
              print("Invalid medicine ID.")
              return
          medicine_ids.append(medication_id)
      else:
          print("Patient has no medications.")
          return
  
  # Confirm route selection
  print("\nConfirm Route:")
  print(f"Employee: {employee_id}")
  print(f"Number of Patients: {num_patients}")
  for i in range(num_patients):
      print(f"\tPatient {i+1}: {patient_ids[i]} - Medicine: {medicine_ids[i]}")
  
  confirmation = input("\nIs this route correct? (y/n): ")
  if confirmation.lower() == 'y':
      # Build route
      for patient_id in patient_ids:
          route.append(database['patients'][patient_id]['coordinates'])
  
      # Add return to base for resupply
      route.insert(0, (0, 0))
      route.append((0, 0))
  
      # Save route to file
      filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_route.txt"
      save_route(route, filename)
      print(f"Route saved successfully as {filename}")
  else:
      print("Route planning cancelled.")
  
def save_route(route, filename):
  with open(filename, 'w') as file:
      file.write("Route:\n")
      for point in route:
          file.write(f"\t- {point}\n")

def validate_prescriber_id(prescriber_id, database):
    # Check if the Prescriber ID exists in the database
    if prescriber_id in database['prescribers']:
        print("Prescriber ID is valid.\t Access granted.")
        time.sleep(3)
        return True
    else:
        print("Invalid Prescriber ID.")
        time.sleep(3)
        return False


def main():


    # Load database at the beginning of the main function
    database = load_database()

    while True:
        display_menu()
        choice = input("\n  Enter your choice (1-6): ")
## Lines 159 - 176 are calling the menu twice. Unneeded. 
            # Implement route planning logic (e.g., user input, file interaction)

      ## Beginning 


        if choice == '1': ## 90% Functionality

          # Route planning functionality
            print("Route planning functionality in progress...")
            database = load_database()
            route_planning(database)

        elif choice == '2': ## 95% Functional
            """
            The route loading functionality should do the following:
            1.) Ask the user to input the route file name.
            2.) open the .txt file with the coordinates. 
            3.) Print out which file is loaded
            4.) Confirm the coordinates by printing out where the robot is headed 

            """
            # Implement route loading logic (e.g., file reading)
            print("Route loading functionality in progress...") ## Beginning 
            route_loading()

        elif choice == '3': ## 100% Functional
            # Implement route history viewing logic (e.g., file reading/display)
            """
            The route history functionality should accomplish the following: 

            1.) Print out the log file that will detail each of of the routes were exceuted.

            """
            route_history_viewing()

            print("Route history viewing functionality in progress...") ## Beginning 

        elif choice == '4': ## 0% functional
            # Implement route execution logic (e.g., sending commands to robot)
            print("Route execution functionality in progress...")
            """
            This functionality will accomplish the following:
                (1.) Run the Ros navigation program in a separate terminal window 
                (1a) Prompt teh user to either run the plan autonomously or  via teleoperation.
                (1b) If the user chooses to run the plan autonomously, the program will launch RVIZ2 and open the ROS code to run the robot in a seperate terminal
                (1c) If the user chooses to run the plan via teleoperation, the program will launch RVIZ2, open the ROS code to run the robot via teleoperatoin in a seperate terminal, and prompt the user to input the coordinates for the navigation plan)
                (2.) Append the route being excuted to the log file. 
                (3.) Request a Valid Percriber ID that is validated via the database before exceuted. 
            """
            execute_route(database, LOADED_ROUTE)
          

        elif choice == '5': ## 75% functional
            # Implement patient data editing logic (e.g., user input, database interaction)
            print("Edit Patient Data functionality in progress...")
            """
            This functionality will accomplish the following: 
                    1.) Access the patient database 
                    2.) Allow the user to "CRUD" -- Create, Read, Update and Delete within that database
                    3.) Output the database information in a way that is user-readable.

            """
            option_5(database)

        elif choice == '6': ## 100% Functional 
          # Exits the program. 
            print("Exiting HMI Interface...")
            break
        else:
            print("Invalid choice. Please enter a number between 1 and 6.")



if __name__ == "__main__":
    main()
