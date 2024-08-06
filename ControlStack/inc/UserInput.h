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

using namespace Hopper_t;

class Command{
public: 
    virtual ~Command() {};
    virtual void update(vector_3t &v) = 0;
    virtual matrix_t getCommand() = 0;
    virtual int getHorizon() const = 0;
    virtual int getStateDim() const = 0;
};

class V3Command : public Command{
public: 
    V3Command();
    int getHorizon() const override { return 1; }
    int getStateDim() const override { return 3; }
    vector_3t command;
    void update(vector_3t &v);
    matrix_t getCommand() {return command;};
};

class SingleIntCommand : public Command {
public: 
    SingleIntCommand(const int horizon);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 2; }
    const float dt = 0.02;
    const double v_max = 0.35;
    void update(vector_3t &v);
    matrix_t getCommand() {return command;};
};

class DoubleIntCommand : public Command {
public: 
    DoubleIntCommand(const int horizon);
    matrix_t command;
    const int horizon;
    int getHorizon() const override { return horizon; }
    int getStateDim() const override { return 4; }
    const float dt = 0.02;
    const double v_max = 0.35;
    const double a_max = 1.0;
    void update(vector_3t &v);
    matrix_t getCommand() {return command;};
};

class UserInput{

public:
    UserInput(){};
    ~UserInput(){};
    // Current state of an axis.
    struct axis_state {
        short x, y;
    };
    vector_3t joystick_command;

    matrix_t singleIntHorizon;

    // simple list for button map
    char buttons[4] = {'X','O','T','S'}; // cross, cricle, triangle, square

    scalar_t increment = 0.001;

    // Reads a joystick event from the joystick device.
    // Returns 0 on success. Otherwise -1 is returned.
    int read_event(int dev, struct js_event *event);

    // get PS4 LS and RS joystick axis information
    size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);

    void getJoystickInput(vector_2t &offsets, std::unique_ptr<Command> &command,
                          scalar_t &dist, std::condition_variable & cv, std::mutex & m);

    static vector_3t keyboardInput();

    // https://stackoverflow.com/questions/41505451/c-multi-threading-communication-between-threads
    // https://stackoverflow.com/questions/6171132/non-blocking-console-input-c
    // MH
    // waits for the user input and asyncronously wait for the input, if everything is okay, it reads the input to the command
    // don't read too much into it
    void getKeyboardInput(std::unique_ptr<Command> &command, std::condition_variable & cv, std::mutex & m);
};

// class StandardUserInput: UserInput {
//     void getJoystickInput(vector_2t &offsets, v3Command &command,
//                           vector_3t &dist, std::condition_variable & cv, std::mutex & m);
// };

// class SingleIntUserInput: UserInput {
//     void getJoystickInput(vector_2t &offsets, singleIntCommand &command,
//                           vector_3t &dist, std::condition_variable & cv, std::mutex & m);
// };

#endif