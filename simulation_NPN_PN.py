import threading
import time
import logging
import subprocess
import socket
import math
import random
import datetime
import argparse
from io import StringIO

# Generate a timestamp for the log filenames
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# Set up logging for before migration
before_data_log_file = f"handover_data_before_migration_{timestamp}.log"
status_log_file = f"handover_status{timestamp}.log"

logging_before_data = logging.getLogger("BeforeMigrationData")
logging_before_data.setLevel(logging.INFO)
before_data_handler = logging.FileHandler(before_data_log_file)
before_data_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logging_before_data.addHandler(before_data_handler)

logging_status = logging.getLogger("MigrationStatus")
logging_status.setLevel(logging.INFO)
status_handler = logging.FileHandler(status_log_file)
status_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logging_status.addHandler(status_handler)

# Set up logging for after migration
after_data_log_file = f"handover_data_after_migration_{timestamp}.log"
tcpdump_log_file = f"tcpdump{timestamp}.pcap"

logging_after_data = logging.getLogger("AfterMigrationData")
logging_after_data.setLevel(logging.INFO)
after_data_handler = logging.FileHandler(after_data_log_file)
after_data_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
logging_after_data.addHandler(after_data_handler)

tcpdump_status = logging.getLogger("TcpDumpStatus")
tcpdump_status.setLevel(logging.INFO)
tcpdump_status_handler = logging.FileHandler(tcpdump_log_file)
tcpdump_status_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
tcpdump_status.addHandler(tcpdump_status_handler)


# Constants
GNODEB1_COORD = (1000, 0)  # gNodeB1 is at 1000 meters (final destination)
GNODEB2_COORD = (0, 0)  # gNodeB2 is at the origin
GNODEB2_COVERAGE_RADIUS = 200  # meters (coverage radius of gNodeB2)

MESSAGE_INTERVAL_PNK3 = 0.5  # seconds (initial interval for PNK3)
MESSAGE_INTERVAL_NPNK3 = 0.002  # seconds (2 milliseconds for NPNK3)



# SSH and server details
PNK3_USER_HOST = 'Pnk3@10.255.44.48'
NPNK3_USER_HOST = 'Npnk3@10.255.32.40'
ECHO_SERVER_PORT = 8080
SSH_KEY_PATH = '/home/RAN/new_key'  # Update this path as necessary

#UE Process's to implement UERANSIM
ue_PN_process= None
ue_NPN_process= None
global new_communication_thread
#TCPDUMP Command to log server Communications
tcpdump_command = f"sudo tcpdump -i any host 10.255.44.48 or host 10.255.32.40 -w {tcpdump_log_file}"

# Shared data structure for RSRP, RSRQ values, and UE position
class SignalData:
    def __init__(self):
        self.rsrp_gnodeb1 = -100  # Initial weak signal for gNodeB1 (UE starts closer to gNodeB2)
        self.rsrp_gnodeb2 = -50  # Initial strong signal for gNodeB2
        self.rsrq_gnodeb1 = -20  # Placeholder for RSRQ
        self.rsrq_gnodeb2 = -3  # Placeholder for RSRQ
        self.ue_position = [0, 0]  # UE starts at gNodeB2
        self.ue_migrated = False
        self.pnk3s_started = False
        self.response_count_before = 0
        self.response_count_after = 0
        self.start_time_before = None
        self.start_time_after = None
        self.stop_communication = False
        self.lock = threading.Lock()

signal_data = SignalData()

def ssh_run_k3s_command(command, user_host):
    ssh_command = f"ssh -i {SSH_KEY_PATH} {user_host} '{command}'"
    try:
        subprocess.run(ssh_command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        logging_status.info(f"Successfully ran command on {user_host}: {command}")
    except subprocess.CalledProcessError as e:
        logging_status.error(f"Failed to run command on {user_host}: {e.stderr.decode().strip()}")

def start_echo_server_service(user_host):
    logging_status.info(f"Starting the echo-server service on {user_host}...")
    ssh_run_k3s_command("sudo k3s kubectl apply -f echo-server-deployment.yaml", user_host)
def delete_echo_server_service(user_host):
    logging_status.info(f"Deleting the echo-server service on {user_host}.")
    ssh_run_k3s_command("sudo k3s kubectl delete service echo-server", user_host)

def run_cleanup_script(user_host):
    logging_status.info(f"Running cleanup script on {user_host}...")
    ssh_command = f"ssh -i {SSH_KEY_PATH} {user_host} './delete_echo_server.sh'"
    try:
        subprocess.run(ssh_command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        logging_status.info(f"Cleanup script executed successfully on {user_host}.")
    except subprocess.CalledProcessError as e:
        logging_status.error(f"Failed to run cleanup script on {user_host}: {e.stderr.decode().strip()}")

def communicate_with_echo_server(socket, message='Hello, Echo Server!'):
    try:
        socket.sendall(message.encode('utf-8'))
        response = socket.recv(1024)
        if not signal_data.ue_migrated:
            signal_data.response_count_before += 1
        else:
            signal_data.response_count_after += 1
    except Exception as e:
        if not signal_data.ue_migrated:
            logging_status.error(f"Error communicating with server: {e}")
        else:
            logging_status.error(f"Error communicating with server: {e}")

def continuous_communication():
    signal_data.start_time_before = time.time()

    # Initialize connection before migration
    current_host = NPNK3_USER_HOST.split('@')[1]
    interval = MESSAGE_INTERVAL_NPNK3
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, 25, 'uesimtun0'.encode())  # Use the specified interface
            s.connect((current_host, ECHO_SERVER_PORT))
            logging_status.info(f"Initial connection established with {current_host}.")

            while True:
                if signal_data.ue_migrated:
                        # Migration occurred; stop the current communication thread
                        logging_status.info("Stopping current communication due to migration.")
                        break

                communicate_with_echo_server(s)
                time.sleep(interval)

    except Exception as e:
        logging_status.error(f"Failed to establish or maintain connection: {e}")


def restart_communication_with_new_server():
    signal_data.start_time_after = time.time()

    current_host = PNK3_USER_HOST.split('@')[1]
    interval = MESSAGE_INTERVAL_PNK3

   
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s_new:
            s_new.setsockopt(socket.SOL_SOCKET, 25, 'uesimtun0'.encode())
            s_new.connect((current_host, ECHO_SERVER_PORT))
            logging_status.info(f"Connection re-established with {current_host} after migration.")
            
            while True:
                if signal_data.stop_communication:
                        logging_status.info("Stopping communication with the new server as UE stopped moving.")
                        break  # Exit the loop and stop communication
                communicate_with_echo_server(s_new)
                time.sleep(interval)
          
            logging_status.info("Stopping UE.")
            ue_PN_process.kill()
            logging_status.info("UE Stopped.")
            exit
    except Exception as e:
        logging_status.error(f"Failed to establish or maintain connection with new server: {e}")


# Calculate the distance between two points
def calculate_distance(coord1, coord2):
    return math.sqrt((coord2[0] - coord1[0]) ** 2 + (coord2[1] - coord1[1]) ** 2)

# Function to generate RSRP and RSRQ values for gNodeB1 and gNodeB2 based on UE distance
def generate_signals():
            ue_distance_from_gnodeb1 = calculate_distance(signal_data.ue_position, GNODEB1_COORD)
            ue_distance_from_gnodeb2 = calculate_distance(signal_data.ue_position, GNODEB2_COORD)
            
            # Random noise to simulate signal variations
            noise_rsrp = random.uniform(-3, 3)  # dB variation
            noise_rsrq = random.uniform(-1, 1)  # dB variation

            # RSRP calculation using logarithmic function for gNodeB1 and gNodeB2
           # RSRP calculation using logarithmic function for gNodeB1 and gNodeB2
            if ue_distance_from_gnodeb1 <= 1000:
                # gNodeB1 is farther, so its signal increases more slowly as we get closer
                signal_data.rsrp_gnodeb1 = -98 + 5 * math.log10(1000 - ue_distance_from_gnodeb1 + 1) + noise_rsrp
            else:
                signal_data.rsrp_gnodeb1 = -100  # Signal drops off outside range

            if ue_distance_from_gnodeb2 <= GNODEB2_COVERAGE_RADIUS:
                # gNodeB2 is closer, so its signal degrades slightly faster
                signal_data.rsrp_gnodeb2 = -60 - 15 * math.log10(ue_distance_from_gnodeb2 + 1) + noise_rsrp
            else:
                signal_data.rsrp_gnodeb2 = -100  # Signal is weak until UE is within 200m

            # RSRQ calculation using exponential function for gNodeB1 and gNodeB2
            if ue_distance_from_gnodeb1 <= 1000:
                # Closer to gNodeB1, the RSRQ is high but degrades as we move away
                signal_data.rsrq_gnodeb1 = -3 - 17 * (1 - math.exp(-ue_distance_from_gnodeb1 / 500)) + noise_rsrq
            else:
                signal_data.rsrq_gnodeb1 = -20  # Minimum value outside range

            if ue_distance_from_gnodeb2 <= GNODEB2_COVERAGE_RADIUS:
                # Closer to gNodeB2, the RSRQ improves but never reaches a perfect value
                signal_data.rsrq_gnodeb2 = -3 - 17 * (1 - math.exp(-ue_distance_from_gnodeb2 / 100)) + noise_rsrq
            else:
                signal_data.rsrq_gnodeb2 = -20  # Minimum value until UE is within 200m

        
            if not signal_data.ue_migrated:
                logging_before_data.info(f"UE Position: {signal_data.ue_position}, gNodeB1 RSRP: {signal_data.rsrp_gnodeb1:.2f}dB, gNodeB2 RSRP: {signal_data.rsrp_gnodeb2:.2f}dB")
                logging_before_data.info(f"UE Position: {signal_data.ue_position}, gNodeB1 RSRQ: {signal_data.rsrq_gnodeb1:.2f}dB, gNodeB2 RSRQ: {signal_data.rsrq_gnodeb2:.2f}dB")
            else:
                logging_after_data.info(f"UE Position: {signal_data.ue_position}, gNodeB1 RSRP: {signal_data.rsrp_gnodeb1:.2f}dB, gNodeB2 RSRP: {signal_data.rsrp_gnodeb2:.2f}dB")
                logging_after_data.info(f"UE Position: {signal_data.ue_position}, gNodeB1 RSRQ: {signal_data.rsrq_gnodeb1:.2f}dB, gNodeB2 RSRQ: {signal_data.rsrq_gnodeb2:.2f}dB")

# Function to simulate UE movement and signal strength
def simulate_ue():
    while True:
    
            # Update UE position (moving along the x-axis towards gNodeB1)
            signal_data.ue_position[0] += UE_SPEED
            
            # Generate signals for the current UE position
            generate_signals()

            ue_distance_from_gnodeb2 = calculate_distance(signal_data.ue_position, GNODEB2_COORD)
            ue_distance_from_gnodeb1 = calculate_distance(signal_data.ue_position, GNODEB1_COORD)
            
            # Start the PNK3S server only once when UE is in the 170 to 200 range
            if 170 <= ue_distance_from_gnodeb2 <= 200 and not signal_data.pnk3s_started:
                start_echo_server_service(PNK3_USER_HOST)
                signal_data.pnk3s_started = True  # Set the flag to ensure it only starts once


            # Check if UE should migrate based on signal strength or GPS coordinates
            if 198 <= ue_distance_from_gnodeb2 <= 200 or (180 <= ue_distance_from_gnodeb2 <= 200 and signal_data.rsrp_gnodeb1 > signal_data.rsrp_gnodeb2):
                migrate_ue()
            elif ue_distance_from_gnodeb2 > 200 and not signal_data.ue_migrated:
                # Force migration even if the signal hasn't fully met the threshold
                migrate_ue()

            if signal_data.ue_position[0] >= 400:  # End simulation at 400 meters
                save_logs_and_throughput()
                signal_data.stop_communication = True
                
                break
            
            time.sleep(UPDATE_INTERVAL)
    # Exit the program after the simulation ends
    logging_status.info("Exiting the simulation...")
    elapsed_time = time.time() - signal_data.start_time_before
    logging_status.info(f"Simulation time {elapsed_time:.3f}")
    exit(0)

# Function to calculate throughput
def calculate_throughput(response_count, start_time):
    if start_time is None:
        return 0
    elapsed_time = time.time() - start_time
    return response_count / elapsed_time if elapsed_time > 0 else 0

# Function to write logs and calculate throughput when the simulation ends
def save_logs_and_throughput():
    # Log throughput information
    throughput_after = calculate_throughput(signal_data.response_count_after, signal_data.start_time_after)
    logging_status.info(f"Throughput after migration: {throughput_after:.2f} responses/second")
    
    logging_status.info("Simulation complete, logs and throughput data saved.")
    try:
        subprocess.run(["sudo", "./nr-cli", "imsi-999990000000001", "-e", "deregister switch-off"], check=True)
        logging_status.info("PN UE configuration turned off.")
    except subprocess.CalledProcessError as e:
        logging_status.error(f"Failed to turn off current UE configuration: {e.stderr.decode().strip()}")

# Function to change UE configuration during migration
def reconfigure_ue():
    global ue_PN_process
    # Turn off the current UE configuration
    logging_status.info("Turning off current UE configuration...")
    try:
        subprocess.run(["sudo", "./nr-cli", "imsi-999990000000001", "-e", "deregister switch-off"], check=True)
        logging_status.info("Current UE configuration turned off.")
        global ue_NPN_process
        ue_NPN_process.kill()
    except subprocess.CalledProcessError as e:
        logging_status.error(f"Failed to turn off current UE configuration: {e.stderr.decode().strip()}")
    # Start the new UE configuration
    logging_status.info("Starting new UE configuration with 'open5gs-uePN.yaml'...")
    try:
        ue_PN_process = subprocess.Popen(["sudo", "./nr-ue", "-c","../config/open5gs-uePN.yaml"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        logging_status.info("New UE configuration started.")
    except subprocess.CalledProcessError as e:
        logging_status.error(f"Failed to start new UE configuration: {e.stderr.decode().strip()}")

# Function to simulate UE migration
def migrate_ue():

    if signal_data.ue_migrated:
        return

    # Calculate and log throughput before migration
    throughput_before = calculate_throughput(signal_data.response_count_before, signal_data.start_time_before)
    logging_status.info(f"Throughput before migration: {throughput_before:.2f} responses/second")

    
    if not signal_data.pnk3s_started:
        start_echo_server_service(PNK3_USER_HOST)
        signal_data.pnk3s_started = True 

    logging_status.info("Migrating UE from gNodeB2 to gNodeB1...")

    # Change the UE configuration
    reconfigure_ue()
    signal_data.ue_migrated = True

    # Stop the previous communication thread
    logging_status.info("Stopping previous communication thread.")
    global communication_thread 
    if communication_thread.is_alive():
        communication_thread.join(timeout=0.35)  

    # Start a new communication thread with the new server
    logging_status.info("Starting new communication thread with the new server.")
    global new_communication_thread 
    new_communication_thread = threading.Thread(target=restart_communication_with_new_server)
    new_communication_thread.start()

    logging_status.info("Migration complete.")

def start_tcpdump():
    try:
        logging_status.info(f"Starting tcpdump with command: {tcpdump_command}")
        tcpdump_process = subprocess.Popen(tcpdump_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return tcpdump_process
    except Exception as e:
        logging_status.error(f"Failed to start tcpdump: {e}")
        return None
def stop_tcpdump(tcpdump_process):
    if tcpdump_process:
        try:
            tcpdump_process.terminate()  # Send SIGTERM to stop tcpdump
            tcpdump_process.wait()  # Wait for the process to exit
            logging_status.info("tcpdump stopped successfully.")
        except Exception as e:
            logging_status.error(f"Failed to stop tcpdump: {e}")

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="UE Simulation Script with configurable parameters.")
    parser.add_argument('--ue_speed', type=float, default=0.20, help="Speed of the UE in meters per second.")
    parser.add_argument('--update_interval', type=float, default=0.02, help="Update interval in seconds.")
    parser.add_argument('--rsrp_threshold', type=float, default=-85, help="RSRP threshold for handover in dB.")

    # Parse arguments
    args = parser.parse_args()

    # Assign parsed arguments to variables
    global UE_SPEED, UPDATE_INTERVAL, RSRP_THRESHOLD
    UE_SPEED = args.ue_speed # meters per second along the x-axis
    UPDATE_INTERVAL = args.update_interval # 20 milliseconds
    RSRP_THRESHOLD = args.rsrp_threshold # dB (example threshold for RSRP to trigger handover)
    
    # Start tcpdump subprocess
    tcpdump_process = start_tcpdump()
    # Start the NPN UE
    
    try:
        ue_NPN_process = subprocess.Popen(["sudo", "./nr-ue", "-c","../config/open5gs-ueNPN.yaml"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        logging_status.info("NPN UE configuration started.")
    except subprocess.CalledProcessError as e:
        logging_status.error(f"Failed to start UE configuration: {e.stderr.decode().strip()}")

    # Start the echo server on NPNK3 before initiating the simulation
    start_echo_server_service(NPNK3_USER_HOST)
    
    # Start both communication and simulation threads simultaneously
    communication_thread = threading.Thread(target=continuous_communication)
    simulation_thread = threading.Thread(target=simulate_ue)
    
   
    # Start both threads
    communication_thread.start()
    simulation_thread.start()
    
    # Wait for both threads to finish

    simulation_thread.join()
    
    # Stop tcpdump subprocess after the simulation ends
    stop_tcpdump(tcpdump_process)
    # Run cleanup script on both host machines
    run_cleanup_script(PNK3_USER_HOST)
    run_cleanup_script(NPNK3_USER_HOST)

    if new_communication_thread.is_alive():   
        new_communication_thread.join(timeout=0.5) 
    exit()

