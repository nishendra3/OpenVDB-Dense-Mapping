#include <iostream>

// include the test functions
#include "tests/test1.hpp"
#include "tests/test2.hpp"
#include "tests/test3.hpp"
#include "tests/test4.hpp"

int main()
{
    // path to the output directory
    std::string path = "/home/nishi/Desktop/sp/submission/out/tmp";
    
    // enum for the test cases
    enum test_cases
    {
        FileIO = 1,
        DataAccess = 2,
        FreeSpaceQuery = 3,
        NearestNeighbour = 4
    };


    // run specific test cases
    int test = test_cases::FileIO;


    // test cases switch statements
    #include "tests/test_cases.hpp"
    

    return 0;
}



