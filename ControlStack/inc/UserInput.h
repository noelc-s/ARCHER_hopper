#ifndef USERINPUT_H
#define USERINPUT_H

// for joystick inputs
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <linux/joystick.h>
#include <stdlib.h>
#include <mutex>
#include <future>
#include <condition_variable>
#include "../inc/Types.h"
#include <thread>

using namespace Hopper_t;

class UserInput{

public:
    UserInput(){joystick_command.setZero(); keyboard_command.setZero();};
    ~UserInput(){};
    // Current state of an axis.
    struct axis_state {
        short x, y;
    };
    vector_4t joystick_command;
    vector_3t keyboard_command;
    const float deadzone = 0.025;

    matrix_t singleIntHorizon;

    // simple list for button map
    char buttons[4] = {'X','O','T','S'}; // cross, cricle, triangle, square

    scalar_t increment = 0.001;

    // Reads a joystick event from the joystick device.
    // Returns 0 on success. Otherwise -1 is returned.
    int read_event(int dev, struct js_event *event);

    // get PS4 LS and RS joystick axis information
    size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);

    void getJoystickInput(vector_2t &offsets,
                          scalar_t &dist, std::condition_variable & cv, std::mutex & m);

    static vector_3t keyboardInput();

    // https://stackoverflow.com/questions/41505451/c-multi-threading-communication-between-threads
    // https://stackoverflow.com/questions/6171132/non-blocking-console-input-c
    // MH
    // waits for the user input and asyncronously wait for the input, if everything is okay, it reads the input to the command
    // don't read too much into it
    void getKeyboardInput(vector_2t &offsets,
                          scalar_t &dist, std::condition_variable & cv, std::mutex & m);
    void cornerTraversal(vector_2t &offsets,
                          scalar_t &dist, std::condition_variable & cv, std::mutex & m);
    void resetKeyboardInput();
};

#endif