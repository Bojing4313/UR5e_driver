import time
import os

PATH = os.getcwd()
os.chdir(PATH)

def count_down_print(count_msg:str, end_msg:str="", count_time:int=3):

    for i in range(count_time, -1, -1):
        print("\r{} ({}s)".format(count_msg,i), end='')
        time.sleep(1)

    print("\r{}      {}".format(end_msg, ' '*len(count_msg)), end='\r')

    if end_msg != "":
        print(f"{end_msg}{'  '*len(count_msg)}")

def dynam_print(count_msg:str, end_msg:str="", count_time:int=3):

    for j in range(count_time+1):
        print(f"\r{count_msg}{'.'*j}", end='')
        time.sleep(0.1)
    #print(f"\r{count_msg}{' '*count_time}", end=f'\r{count_msg}')
    #print(f"\r{count_msg}", end='')
    time.sleep(0.1)
    print(f"\r{' '*(count_time + len(count_msg))}", end='\r')
    
    if end_msg != "":
        print(f"{end_msg}{' '*len(count_msg)}")

"""def dynam_print(count_msg:str, end_msg:str="", count_time:int=3):

    print(count_msg, end='')
    for i in range(count_time):
        print(".", end='', flush=True)
        time.sleep(1)

    print("\r{}{}".format(end_msg, '   '*len(count_msg)), end='\r')"""

def log_print(msg):

    os.chdir(PATH)

    with open ("Status.txt", "r") as f:
        states = f.readlines()

    try:
        left_cell = int(states[-1][-3]) #same to read_cell_nr()
        time.sleep(0.01)
    except ValueError:
        left_cell = None

    with open ("Status.txt", "a") as f:
        print(time.strftime("\n[Time: %d/%m/%Y, %H:%M:%S];\t{};\tCells remaining: [{}]".format(msg, left_cell), time.localtime()), file=f)

    print(time.strftime("[Time: %d/%m/%Y, %H:%M:%S];\t{}".format(msg), time.localtime()))

def update_cell_nr(current_cell_nr):

    os.chdir(PATH)

    with open("Status.txt", "a") as f:
        print(time.strftime("\n[Time: %d/%m/%Y, %H:%M:%S];\tRemaining cell updatded: [{}]".format(current_cell_nr), time.localtime()), file=f)

    print(time.strftime("[Time: %d/%m/%Y, %H:%M:%S];\tRemaining cell updatded: [{}]".format(current_cell_nr), time.localtime()))

def read_cell_nr():

    os.chdir(PATH)

    with open ("Status.txt", "r") as f:
        states = f.readlines()

    return int(states[-1][-3])
