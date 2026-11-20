#include "buildInfo.h"
#include "launcher.h"

#include <iostream>

int main(int argc, const char* argv[])
{
    std::cout << meshlib::app::buildInformation() << "\n\n";
    return meshlib::app::launcher(argc, argv);;
}
