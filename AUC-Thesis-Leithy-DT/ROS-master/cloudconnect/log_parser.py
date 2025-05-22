def calculate_delay(log_dict1, log_dict2):
    """Calculates the delay between two maps for each sequence number."""
    delays = []
    max = []
    for seq in log_dict1:
        if seq in log_dict2:
            delay = log_dict2[seq] - log_dict1[seq]
            if delay > 0.1:
                max.append(delay)
            else: 
                delays.append(delay)
    # print("Max delay:", max)
    return delays, max

def calculate_average_delay(delays):
    """Calculates the average delay."""
    if delays:
        return sum(delays) / len(delays)
    else:
        return None
    
def read_log_file(file_path):
    # Initialize an empty dictionary
    log_dict = {}
    
    # Open the file and read its contents
    with open(file_path, 'r') as file:
        #read the file and remove any "Disconnected from the server" or 'Successfully connected to the server'
        file = [line for line in file if 'Disconnected from the server' not in line and 'Successfully connected to the server' not in line]
        # Iterate over each line in the file
        for line in file:
            # Split the line into key and value based on the colon (":") separator
            
            parts = line.strip().split(' ')
            # print(parts)
            seq = int(parts[1])
            time = float(parts[3])
            # Add the key-value pair to the dictionary
            log_dict[seq] = time
            
    return log_dict

# Path to the log file
digital_log_path = 'output.log'
physical_log_path = 'turtlebot3_logs/output.log'

digital1_log_path = 'output1.log'
physical1_log_path = 'turtlebot3_logs/output1.log'
# Read the contents of the log file into a dictionary
digital_log = read_log_file(digital_log_path)
physical_log = read_log_file(physical_log_path)

digital1_log = read_log_file(digital1_log_path)
physical1_log = read_log_file(physical1_log_path)
# Print the dictionary

delays, max = calculate_delay(physical_log, digital_log)
average_delay = calculate_average_delay(delays)
average_max_delay = calculate_average_delay(max)

delays1, max1 = calculate_delay(physical1_log, digital1_log)
average_delay1 = calculate_average_delay(delays1)
average_max_delay1 = calculate_average_delay(max1)

avg_max = (average_max_delay1+average_max_delay)/2
avg = (average_delay1+average_delay)/2


print("Max delay:", avg_max)
# print("Max delay1:", average_max_delay1)

print("Average delay:", avg)
# print("Average delay1:", average_delay1)


# for seq, time in physical_log.items():
#     print(f"Sequence: {seq}, Time: {time}")