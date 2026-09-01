#include <gtest/gtest.h>

#include "app/buildInfo.h"

using meshlib::app::buildInformation;

TEST(BuildInfoTest, containsBuildMetadata)
{
    const auto information = buildInformation();

    EXPECT_NE(information.find("Tessellator "), std::string::npos);
    EXPECT_NE(information.find("Compilation date: "), std::string::npos);
    EXPECT_NE(information.find("Git commit: "), std::string::npos);
    EXPECT_NE(information.find("Compiler: "), std::string::npos);
    EXPECT_NE(information.find("Build type: "), std::string::npos);
    EXPECT_NE(information.find("Compilation flags: "), std::string::npos);
    EXPECT_NE(information.find("Compilation flags (Debug): "), std::string::npos);
    EXPECT_NE(information.find("Compilation flags (Release): "), std::string::npos);
}
