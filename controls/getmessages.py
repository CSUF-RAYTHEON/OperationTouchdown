from pymavlink import mavutil
def get_messages(master, msg_types: list) -> dict:
    latest_msgs = {}
    while True:
        # Pymavlink will automatically ignore/cache anything not in your list
        msg = master.recv_match(type=msg_types, blocking=False)
        if not msg:
            break # The serial buffer is completely empty!
            
        # Overwrite the dictionary entry so we only keep the absolute freshest one
        latest_msgs[msg.get_type()] = msg 
        
    return latest_msgs