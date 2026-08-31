#include "buildInfo.h"

#include "tessellatorBuildInfo.h"

#include <sstream>

namespace meshlib::app {

std::string buildInformation()
{
    std::ostringstream output;
    output
        << "Tessellator " << buildInfo::programVersion << '\n'
        << "Compilation date: " << __DATE__ << ' ' << __TIME__ << '\n'
        << "Git commit: " << buildInfo::gitCommit << '\n'
        << "Compiler: " << buildInfo::compiler << '\n'
        << "Build type: " << buildInfo::buildType << '\n'
        << "Compilation flags: " << buildInfo::compilationFlags << '\n'
        << "Compilation flags (Debug): " << buildInfo::compilationFlagsDebug << '\n'
        << "Compilation flags (Release): " << buildInfo::compilationFlagsRelease;
    return output.str();
}

}
