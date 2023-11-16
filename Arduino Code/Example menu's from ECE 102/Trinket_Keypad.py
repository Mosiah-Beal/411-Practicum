# Keypad Trinket - Compares inputs against limited library of valid commands.
# Sends response to Dispatch trinket depending on what is input.
import gc
import time
import board
import pwmio
from digitalio import DigitalInOut, Direction, Pull
from analogio import AnalogOut
from adafruit_mcp230xx.mcp23017 import MCP23017
#print(gc.mem_free())
gc.collect()


def get_voltage(pin):
    """Reads tha analog voltage at specified pin and converts it to the equivalent voltage in a 0 - 3.3V range"""
    return (pin.value * 3.3) / 65535


def send_voltage(voltage):
    """Takes a target voltage from a range of 0 - 3.3V and converts it to an equivalent analog voltage."""
    global toggle_pin
    #print(int((voltage / 3.3) * 65535))
    toggle_pin.value = int((voltage / 3.3) * 65535)
    time.sleep(0.1)
    toggle_pin.value = 0


def pressed_keys(key_sleep=0.05):
    """An array containing all detected keys that are pressed from the initalized
    list-of-lists passed in during creation"""
    # make a list of all the keys that are detected
    global keys, rows, cols, piezo
    pressed = []

    for row, row_pin in enumerate(rows):
        # set one row low at a time
        row_pin.value = False

        # check the column pins, which ones are pulled down
        for col, val in enumerate(cols):
            if not val.value:
                time.sleep(key_sleep)
                pressed.append(keys[row][col])
        # reset the row pin to be high
        row_pin.value = True
    return pressed
gc.collect()

def check_keypad():
    """Scans the keypad for inputs, looking for the # character to denote the end
    of the input."""
    # Initialize string with a placeholder character to avoid indexing errors
    # print('inside functun')
    entered_password = ""
    while True:
        output = pressed_keys()
        # wait for a response
        if not output:
            #del output
            continue
        elif output[-1] == '#':
            del output
            piezo.duty_cycle = 65535 // 2
            time.sleep(0.05)
            piezo.duty_cycle = 0
            return entered_password
        else:
            for x in output:
                entered_password += str(x)
                piezo.duty_cycle = 65535 // 2
                time.sleep(0.05)
                piezo.duty_cycle = 0
            print(entered_password)
            del output
        gc.collect()


gc.collect()

# Feedback for bad password
#led = DigitalInOut(board.D13)
#led.direction = Direction.OUTPUT

# Define A0 to be analog output to send responses
# to dispatch which will rely the info to the other
# trinkets as needed
toggle_pin = AnalogOut(board.A0) # pin D1

# speaker for key press feedback
piezo = pwmio.PWMOut(board.D3, frequency=500, variable_frequency=True)

# Create an instance of MCP23017 class
i2c = board.I2C() # pins D0 and D2
mcp = MCP23017(i2c)

# Assigning names to associated pins on IC2 expander
# Make the column pins pulled up inputs
col4 = mcp.get_pin(12)
col4.direction = Direction.INPUT
col4.pull = Pull.UP

col3 = mcp.get_pin(11)
col3.direction = Direction.INPUT
col3.pull = Pull.UP

col2 = mcp.get_pin(10)
col2.direction = Direction.INPUT
col2.pull = Pull.UP

col1 = mcp.get_pin(9)
col1.direction = Direction.INPUT
col1.pull = Pull.UP

col0 = mcp.get_pin(8)
col0.direction = Direction.INPUT
col0.pull = Pull.UP


# Make the row pins active low outputs
row4 = mcp.get_pin(3)
row4.direction = Direction.OUTPUT
row4.value = True

row2 = mcp.get_pin(4)
row2.direction = Direction.OUTPUT
row2.value = True

row3 = mcp.get_pin(5)
row3.direction = Direction.OUTPUT
row3.value = True

row1 = mcp.get_pin(6)
row1.direction = Direction.OUTPUT
row1.value = True

row0 = mcp.get_pin(7)
row0.direction = Direction.OUTPUT
row0.value = True

# setting up key matrix
cols = [col0, col1, col2, col3, col4]
rows = [row0, row1, row2, row3, row4]
keys = (
    (1, 2, 3, "", ""),
    (4, 5, 6, "", ""),
    (7, 8, 0, "", ""),
    ("", "", 9, "", ""),
    ("", "", "", "*", "#"),
)

master_pass = 123  # The default password
armed = False

print("Finalize your input with the '#' key\n")
while True:
    print("Set a new password (1)\n"
          "Initialize the security system (2)\n"
          "Quit the program (3)\n")
    entry = check_keypad()

    # If the first button press is 1: set a password
    if entry == '1':
        del entry
        send_voltage(2.2)
        print("The new password must be 3 digits long\n")
        while True:
            temp_pass = check_keypad()
            # print(len(temp_pass))
            if (len(temp_pass) != 3) or ('*' in temp_pass):
                send_voltage(2.4)
                del temp_pass
                print("Password must be 3 digits!")
                temp_pass = check_keypad()
            else:
                send_voltage(2.6)
                master_pass = temp_pass
                del temp_pass
                break
        print('The new password is now:', master_pass, '\n')

    # If they press 2, initialize the security system
    elif entry == '2':
        send_voltage(1.2)
        print('The password is', master_pass)
        print("Initializing security system in Disarmed mode")
        break

    # If they press 3, shut down the system
    elif entry == '3':
        send_voltage(1.4+0.115)
        print("Shutting down")
        break
    else:
        send_voltage(2.8)
        print("That option doesn't exist\n")

    gc.collect()
    time.sleep(0.1)
# not shut down
if entry != '3':
    del entry
    # scan for inputs
    print("Enter *1 to toggle external sensors\n"
              "Enter *2 to toggle internal sensors\n"
              "Enter *3 to toggle lights\n"
              "Enter ****5 to shut down")
    while True:
        gc.collect()
        time.sleep(0.01)
        if not pressed_keys():
            continue
        send_voltage(2.0+0.115) # stop timer if active
        command = check_keypad()

        if command == str(master_pass):
            del command
            send_voltage(1.0)  # toggle arm/disarm
            send_voltage(1.6 + 0.115) # dispatch will interpert this as a good
                                      # pass only if a sensor was tripped
            if armed:
                print("Disarming\n")
            else:
                print("Arming\n")
            armed = not armed

        elif command == '*1':
            del command
            send_voltage(0.4)  # toggle external
            print('Toggling external sensors\n')

        elif command == '*2':
            del command
            send_voltage(0.6)  # toggle internal
            print('Toggling internal sensors\n')

        elif command == '*3':
            del command
            send_voltage(0.8)  # toggle led
            print('Toggling the lights\n')

        elif command == '****5':
            del command
            send_voltage(1.4 + 0.115) # shut down system
            print('shutting down')
            break
        else:
            del command
            print("Not a valid input\n")
            send_voltage(1.8+0.115) # dispatch will interpert this as a bad
                                    # pass only if a sensor was tripped
            #led.value = True # Feedback that user gave a invalid command
            #time.sleep(0.1)
            #led.value = False


#print("Enter *1 to toggle external sensors\n"
#              "Enter *2 to toggle internal sensors\n"
#             "Enter *3 to toggle lights\n"
#            "Enter ****5 to shut down")

