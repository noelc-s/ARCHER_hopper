// for joystick inputs
#include "../inc/UserInput.h"

// Reads a joystick event from the joystick device.
// Returns 0 on success. Otherwise -1 is returned.
int UserInput::read_event(int dev, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(dev, event, sizeof(*event)); // read bytes sent by controller

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

// get PS4 LS and RS joystick axis information
size_t UserInput::get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    /* hard code for PS4 controller
       Left Stick:  +X is Axis 0 and right, +Y is Axis 1 and down
       Right Stick: +X is Axis 3 and right, +Y is Axis 4 and down
    */
    size_t axis;

    // Left Stick (LS)
    if (event->number == 0 || event->number == 1)
    {
        axis = 0; // arbitrarily call LS Axis 0
        if (event->number == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    // Right Stick (RS)
    else
    {
        axis = 1; // arbitrarily call RS Axis 1
        if (event->number == 3)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

void UserInput::getJoystickInput(vector_2t &offsets,
                                 scalar_t &reset, std::condition_variable &cv, std::mutex &m)
{
    vector_3t input;
    input.setZero();
    std::chrono::seconds timeout(50000);
    const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    // if only one joystick input, almost always "/dev/input/js0"
    device = "/dev/input/js0";

    // joystick device index
    js = open(device, O_RDONLY);
    if (js == -1)
        perror("Could not open joystick");
    else
        std::cout << "joystick connected" << std::endl;

    // scaling factor (joysticks vals in [-32767 , +32767], signed 16-bit)
    //  ... changed for RL testing ... scale to +- 1
    double joystick_max = 32767.;

    /* This loop will exit if the controller is unplugged. */
    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {

        // moving a joystick
        case JS_EVENT_AXIS:
            axis = get_axis_state(&event, axes);
            if (axis == 0)
            {
                joystick_command[0] = axes[axis].x / joystick_max;
                joystick_command[1] = -axes[axis].y / joystick_max;
                if (abs(joystick_command[0]) < deadzone)
                { // Deadzone
                    joystick_command[0] = 0.0;
                }
                if (abs(joystick_command[1]) < deadzone)
                { // Deadzone
                    joystick_command[1] = 0.0;
                }
            }
            if (axis == 1)
            {
                joystick_command[2] = axes[axis].x / joystick_max;
                joystick_command[3] = -axes[axis].y / joystick_max;

                if (abs(joystick_command[2]) < deadzone)
                { // Deadzone
                    joystick_command[2] = 0.0;
                }
                if (abs(joystick_command[3]) < deadzone)
                { // Deadzone
                    joystick_command[3] = 0.0;
                }
            }
            
            break;
        // pressed a button
        case JS_EVENT_BUTTON:
            if (event.number == 5 && event.value == 1)
            {
                std::cout << "reset" << std::endl;
                reset = 1;
            }
            if (event.number == 4 && event.value == 1)
            {
                std::cout << "Killed due to user input" << std::endl;
                exit(1);
            }
            if (event.number == 8 && event.value == 1)
            {
                increment *= 10;
                std::cout << "Increasing resolution to: " << increment << std::endl;
            }
            if (event.number == 9 && event.value == 1)
            {
                increment /= 10;
                std::cout << "Decreasing resolution to: " << increment << std::endl;
            }
            // can do something cool with buttons
            if (event.number == 0 && event.value == 1)
                offsets[1] -= increment;
            if (event.number == 1 && event.value == 1)
                offsets[0] += increment;
            if (event.number == 2 && event.value == 1)
                offsets[1] += increment;
            if (event.number == 3 && event.value == 1)
                offsets[0] -= increment;
            std::cout << "Offsets (r, p): " << offsets.transpose() << std::endl;
            break;

        // ignore init events
        default:
            break;
        }
    }

    close(js);
}

vector_3t UserInput::keyboardInput()
{

    vector_3t input;
    std::string line;
    getline(std::cin, line);

    std::istringstream iss(line);
    int pos = 0;
    scalar_t num;

    while (iss >> num)
    {
        input[pos] = num;
        pos++;
    }

    return input;
}

// https://stackoverflow.com/questions/41505451/c-multi-threading-communication-between-threads
// https://stackoverflow.com/questions/6171132/non-blocking-console-input-c
// MH
// waits for the user input and asyncronously wait for the input, if everything is okay, it reads the input to the command
// don't read too much into it
void UserInput::getKeyboardInput(vector_2t &offsets,
                          scalar_t &dist, std::condition_variable &cv, std::mutex &m)
{
    vector_3t input;
    input.setZero();

    std::chrono::seconds timeout(50000);

    while (1)
    {
        std::future<vector_3t> future = std::async(keyboardInput);

        if (future.wait_for(timeout) == std::future_status::ready)
            keyboard_command = future.get();
            joystick_command(0) = keyboard_command(0);
            joystick_command(1) = keyboard_command(1);
            joystick_command(2) = keyboard_command(2);
        std::cout << keyboard_command << std::endl;
    }
}

void UserInput::resetKeyboardInput()
{
    keyboard_command.setZero();
}

void UserInput::cornerTraversal(vector_2t &offsets,
                          scalar_t &dist, std::condition_variable &cv, std::mutex &m)
{
    vector_3t input;
    input.setZero();

    std::chrono::seconds timeout(50000);
    scalar_t sign = 1;

    while (1)
    {
        std::future<vector_3t> future = std::async(keyboardInput);

        if (future.wait_for(timeout) == std::future_status::ready)
            joystick_command(0) = -0.7 + -sign*1.5;
            joystick_command(1) = 0;
            joystick_command(2) = 0;
            sign *= -1;
    }
}