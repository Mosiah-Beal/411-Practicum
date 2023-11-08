# Dispatch Trinket / Keypad Interperter
# Reads the signals being sent and prints messages to screen.
import time
import board
import gc
from digitalio import DigitalInOut, Direction, Pull
from analogio import AnalogIn, AnalogOut


# Initial States
internal_sensor = False # photocells are toggled off
external_sensor = False # reed switches are toggled off
led_state = False # lights are off
alarm = False # alarm is off
armed = False # system is disarmed
start = False # system hasn't been initialized by keypad
shut_down = False # system hasn't been shut down
safe = True # nothing has tripped yet
correct = False # They haven't successfully disarmed the system while the alarm is running
stop_time = False # They haven't pressed any keys yet
rejected = False # They haven't submitted a bad password


# pins
sensor_pin = AnalogIn(board.A3)  # Communication in from sensors
keypad_in_pin = AnalogIn(board.A4) # Communications in from keypad
toggle_pin = AnalogOut(board.A0) # Instructions out to sensors

feedback_light = DigitalInOut(board.D13) # Inform user that the password submitted is incorrect
feedback_light.direction = Direction.OUTPUT

warning_light = DigitalInOut(board.D2) # Inform user of sensor trip
warning_light.direction = Direction.OUTPUT
warning_light.value = False


def get_voltage(pin):
    """Reads tha analog voltage at specified pin and converts it to the equivalent voltage in a 0 - 3.3V range"""
    return (pin.value * 3.3) / 65535


def send_voltage(voltage, sleep=0.1):
    """Takes a target voltage from a range of 0 - 3.3V and converts it to an equivalent analog voltage."""
    global toggle_pin
    # print(voltage)
    toggle_pin.value = int((voltage / 3.3) * 65535)
    time.sleep(sleep)
    toggle_pin.value = 0


# talk to other trinkets (not keypad)
def input_signal(bypass=False, test_signal=0):
    global sensor_pin, internal_sensor, external_sensor
    if bypass:
        sensor = test_signal
    else:
        sensor = get_voltage(sensor_pin)
        #print(sensor)

    # If no signals are sent, do nothing
    if sensor < 0.5:
        # print('banana')
        pass

    # Internal sensors have tripped
    elif (0.5 <= sensor <= 0.7) and internal_sensor and armed:
        print("The motion sensor has been tripped!")
        check_pass()

    # External sensors have tripped
    elif (0.8 <= sensor <= 1.0) and external_sensor and armed:
        print("The external sensors have been tripped!")
        check_pass()

    # Startup phase
    elif 1.1 <= sensor <= 1.3:
        global start
        start = True
        print('Starting systems.')

    # Quit the program
    elif 1.4 <= sensor <= 1.6:
        print('Shutting down.')
        global shut_down
        shut_down = True

    # Toggle the internal sensors
    elif 1.7 <= sensor <= 1.9:
        print('Toggling internal sensors due to dispatch trinket.')
        pass

    # Toggle the external sensors
    elif 2.0 <= sensor <= 2.2:
        print('Toggling external sensors due to dispatch trinket.')
        pass

    # Toggle the mode of internal sensors (optional, may replace if deemed too difficult/long)
    elif 2.3 <= sensor <= 2.5:
        #print('internal mode is toggling')
        pass

    # Toggle the leds
    elif 2.6 <= sensor:
        print('Toggling leds due to dispatch trinket.')
        pass

    #elif 2.8 <= sensor:  #keypad doesn't recieve signals
        #print('sent warning to keypad')
        #pass

    # Sound the alarm
    elif 2.8 <= sensor:
        print("The alarm has been raised!")
        global alarm
        alarm = not alarm


def keypad_signal():
    global keypad_in_pin, armed
    sensor = get_voltage(keypad_in_pin)
    #print(sensor)

    # silence
    if sensor <= 0.3:
        pass

    # toggle external
    elif sensor <= 0.5:
        if armed:
            print("You need to disarm the system first.")
        else:
            send_voltage(2.1)
            global external_sensor
            external_sensor = not external_sensor
            if external_sensor:
                print("External sensors have been toggled on by the keypad.")
            else:
                print('External sensors have been toggled off by the keypad.')
            '''print("Enter *1 to toggle external sensors\n"
              "Enter *2 to toggle internal sensors\n"
              "Enter *3 to toggle lights\n"
              "Enter 5 to shut down\n")'''

    # toggle internal
    elif sensor <= 0.7:
        if armed:
            print("You need to disarm first.")
        else:
            send_voltage(1.8)
            global internal_sensor
            internal_sensor = not internal_sensor
            if internal_sensor:
                print("The motion sensor has been toggled on by the keypad.")
            else:
                print('The motion sensor has been toggled off by the keypad.')
        '''print("Enter *1 to toggle external sensors\n"
              "Enter *2 to toggle internal sensors\n"
              "Enter *3 to toggle lights\n"
              "Enter 5 to shut down\n")'''

    # toggle led
    elif sensor <= 0.9:
        send_voltage(2.5)
        global led_state
        led_state = not led_state
        if led_state:
            print("The lights have been turned on by the keypad.")
        else:
            print('The lights have been turned off by the keypad.')
        '''print("Enter *1 to toggle external sensors\n"
              "Enter *2 to toggle internal sensors\n"
              "Enter *3 to toggle lights\n"
              "Enter 5 to shut down\n") '''
        #testing purposes
        #check_pass()

    # Arm/Disarm the system
    elif sensor <= 1.1:
        global armed, safe, correct
        if not safe:  # if a sensor has tripped, then this means the entered the correct password
            correct = True
            armed = False
            #print("The correct password has been entered by the keypad, alarm has been canceled")
        else: # not tripped behavior
            armed = not armed
            if armed:
                print('The system has been armed.')
            else:
                print("The system has been disarmed.")
        '''print("Enter *1 to toggle external sensors\n"
              "Enter *2 to toggle internal sensors\n"
              "Enter *3 to toggle lights\n"
              "Enter 5 to shut down\n")'''

    # Start initialization
    elif sensor <= 1.3:
        global start
        start = True
        send_voltage(1.2, 0.2)
        print('Initializing sensors.')
        pass

    # shutdown
    elif sensor <= 1.5:
        global shut_down
        shut_down = True
        send_voltage(1.5)
        print('Shutting down the security system.')

    # good pass - handled by conditional in arm/disarm.
    elif sensor <= 1.7:
        pass
        #print('good pass', sensor)
        #timed_pass = '5'
        #global finished
        #finished = True

    # bad pass
    elif sensor <= 1.9:
        # print('The password submitted is incorrect.')
        timed_pass = '6'
        global rejected
        rejected = True


    # stop timer
    elif sensor <= 2.1:
        global stop_time
        if not stop_time:
            print('Response recieved in time, stopping any timers.')
            stop_time = True
        else:
            '''print("Enter *1 to toggle external sensors\n"
                  "Enter *2 to toggle internal sensors\n"
                  "Enter *3 to toggle lights\n"
                  "Enter 5 to shut down\n")'''
            pass

    # sometimes we would be able to recieve 3.3V from the keypad trinket, but since it wasn't reliable
    # we ended up scrapping it.
    '''elif sensor <= 2.3:
        print('The new password must be 3 digits long')

    elif sensor <= 2.5:
        print('Invalid password!\nThe new password must be 3 digits!')

    elif sensor <= 2.7:
        print('Password has been saved')

    elif sensor <= 2.9:
        print("Error: Option doesn't exist")
        print("Set a new password (1)\n"
              "Initialize the security system (2)\n"
              "Quit the program (3)\n")'''


def check_pass():
    print("You have 10 seconds to enter the password before the alarm is raised.")
    global safe, stop_time, rejected, correct, alarm, warning_light
    warning_light.value = True
    time.sleep(0.5)
    # send_voltage(2.8)
    time.sleep(0.1)
    start_time = time.monotonic()  # start timer
    counter = 0
    # ensure the keypad response states are properly initialized before entering loop
    safe = False
    stop_time = False
    rejected = False
    correct = False
    while True:
        keypad_signal()
        now = time.monotonic()
        timer = now - start_time  # NOTICE, a one time timer! Change to a resettable timer?
        if (timer >= 10) and not stop_time: # if more than 10 seconds pass without a response from the keypad
            print("You took too long, raising the alarm.")
            send_voltage(3.0, 0.3) # raise alarm
            break
        elif correct: # if they enter the password correctly
            print('You entered the correct password, disarming the system')
            send_voltage(1.1)
        elif rejected: # if the password is wrong
            counter += 1
            rejected = False
            print("You have entered the wrong code", counter, "times.")
            feedback_light.value = True
            time.sleep(0.1)
            feedback_light.value = False
            continue
        elif counter == 4:
            print('You failed too many times, raising alarm')
            send_voltage(3.0, 0.3) # raise alarm
        else:
            continue
        break
    if correct:
        print('Successfully disarmed.')
    else:
        print("Resetting alarm.")
        time.sleep(5)
        alarm = False
        #print("Alarm reset")
        send_voltage(3.0, 0.3) # turn alarm back off
    safe = True
    rejected = False
    correct = False
    warning_light.value = False

# Live code
print("Set a new password (1)\n"
      "Initialize the security system (2)\n"
      "Quit the program (3)\n")
while not start and not shut_down:
    keypad_signal()
    gc.collect()

    time.sleep(0.1)

while not shut_down:
    # check signals from other
    input_signal()
    gc.collect()

    keypad_signal()
    gc.collect()

    time.sleep(0.1)

print("Successfully shut down")
