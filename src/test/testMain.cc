#include "gtest/gtest.h"
int main(int argc, char **argv) {
    if (argc > 1 ) {
        int i = 1;
        std::string classNames = "";
        while (argv[i]) {
            classNames += argv[i];
            classNames += ".*:";
            ++i;
        }
        ::testing::GTEST_FLAG(filter) = classNames;

    }
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
