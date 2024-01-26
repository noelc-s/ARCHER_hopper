#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>

class UnitTest{

public:
    UnitTest(){};
    ~UnitTest(){};

    void enterValues(const std::string& prompt, std::vector<double>& values);
    std::vector<std::vector<double>> runUnitTest(char initializeUnitTest);
    void rewriteValues(std::vector<std::vector<double>> unitTestInputs, std::vector<double> &p0, std::vector<double> &v0, std::vector<double> &rpy0, std::vector<double> &w0);

};