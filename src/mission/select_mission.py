import os
import sys

def print_title():
    title = """
███╗   ███╗██╗██╗     ██╗████████╗ █████╗ ██████╗ ██╗   ██╗
████╗ ████║██║██║     ██║╚══██╔══╝██╔══██╗██╔══██╗╚██╗ ██╔╝
██╔████╔██║██║██║     ██║   ██║   ███████║██████╔╝ ╚████╔╝ 
██║╚██╔╝██║██║██║     ██║   ██║   ██╔══██║██╔══██╗  ╚██╔╝  
██║ ╚═╝ ██║██║███████╗██║   ██║   ██║  ██║██║  ██║   ██║   
╚═╝     ╚═╝╚═╝╚══════╝╚═╝   ╚═╝   ╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   
                                                           
        ██████╗ ██████╗  ██████╗ ███╗   ██╗███████╗        
        ██╔══██╗██╔══██╗██╔═══██╗████╗  ██║██╔════╝        
        ██║  ██║██████╔╝██║   ██║██╔██╗ ██║█████╗          
        ██║  ██║██╔══██╗██║   ██║██║╚██╗██║██╔══╝          
        ██████╔╝██║  ██║╚██████╔╝██║ ╚████║███████╗        
        ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
Copyright© 2024 Ditya Garda Nugraha                     
    """
    print(title)

# Define the base directory of your mission scripts
base_dir = os.path.expanduser('~/catkin_ws/src/drone_mission/src/mission')

def get_mission_command(script_name):
    try:
        # Construct the full path to the script
        script_path = os.path.join(base_dir, script_name)
        
        # Check if the script exists
        if not os.path.isfile(script_path):
            print(f"Script not found: {script_path}")
            return None
        
        # Construct the command to run the script
        command = f"python3 {script_path}"
        
        return command
    
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

def main():
    print_title()
    print("Pilih salah satu misi yang ingin dijalankan::")
    print("1. Delivery")
    print("2. Bombing")
    print("3. Firing")

    mission_choice = input("Ketik angka sesuai dengan misi yang anda pilih kemudian Enter :")

    if mission_choice == '1':
        script_name = "delivery_mission.py"
    elif mission_choice == '2':
        script_name = "bombing_mission.py"
    elif mission_choice == '3':
        script_name = "firing_mission.py"
    else:
        print("Invalid choice. Exiting.")
        sys.exit(1)

    command = get_mission_command(script_name)

    if command:
        print("\nCopy dan Paste script dibawah untuk memulai misi")
        print(f"\nsource ~/catkin_ws/devel/setup.bash && {command}")
        print("\nBerhati - hatilah ketika menjalankan misi otonom. Selalu siapkan skenario untuk menjalankan drone secara manual dengan Remote Control. Semoga Sukses!")

if __name__ == "__main__":
    main()

