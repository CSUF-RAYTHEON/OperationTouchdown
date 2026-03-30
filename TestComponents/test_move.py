# This code tests getting the local position and storing those values to the variables in the global variable files,
# we calculate our custom local positioning and store the value in a global variable. We constatly poll and print the 
# local position values until we stop the script, this is to ensure that we are able to continuously read the local position
# and see that the values change as we move the drone around.
from pymavlink import mavutil
import global_variables
import time

def test_takeoff(master, height):
    print(f"Entered test_takeoff() for Target System: {master.target_system} & Target Component: {master.target_component}")


def test_move_relative(master, x, y, z):
    print(f"Entered test_move_relative() for Target System: {master.target_system} & Target Component: {master.target_component}")
