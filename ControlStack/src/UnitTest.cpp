#include "../inc/UnitTest.h"

void UnitTest::enterValues(const std::string& prompt, std::vector<double>& values) {
    std::cout << prompt << " (e.g., 4.2 2.4 3.5) and press Enter." << std::endl;
    std::cout << "Alternatively, press Enter if you do not want to change." << std::endl;

    std::string userInput;
    std::getline(std::cin, userInput);

    if (!userInput.empty()) {
        std::istringstream iss(userInput);
        double value;

        while (iss >> value) {
            values.push_back(value);
        }
    }
}

std::vector<std::vector<double>> UnitTest::runUnitTest(char initializeUnitTest) {

    std::vector<std::vector<double>> unitTestInputs(4);
    
    if (initializeUnitTest == 'y') {
        std::cout << "Unit Test Initialized" << std::endl;

        std::vector<double> position;
        enterValues("Enter the Initial Position (x, y, z)", position);
        unitTestInputs[0] = position;

        std::vector<double> velocity;
        enterValues("Enter the Initial Velocity (xdot, ydot, zdot)", velocity);
        unitTestInputs[1] = velocity;

        std::vector<double> orientation;
        enterValues("Enter the Initial Orientation (r, p , y)", orientation);
        unitTestInputs[2] = orientation;

        std::vector<double> angular_velocity;
        enterValues("Enter the Initial Angular Velocity (rdot, pdot, ydot)", angular_velocity);
        unitTestInputs[3] = angular_velocity;
        
    } else {
        std::cout << "Will continue with the default initial conditions." << std::endl;
    }
    return unitTestInputs;
}

void UnitTest::rewriteValues(std::vector<std::vector<double>> unitTestInputs, std::vector<double> &p0, std::vector<double> &v0, std::vector<double> &rpy0, std::vector<double> &w0){
    for(int i = 0; i<unitTestInputs.size(); i++){
        if(unitTestInputs[i].empty()){
            continue;
        }
        else{
            switch(i){
                case 0:
                    p0 = unitTestInputs[i];
                    break;
                case 1:
                    v0 = unitTestInputs[i];
                    break;
                case 2:
                    rpy0 = unitTestInputs[i];
                    break;
                case 3:
                    w0 = unitTestInputs[i];
                    break;
                default:
                    continue;
            }

        }
    }
}