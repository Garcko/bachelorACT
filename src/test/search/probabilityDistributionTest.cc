#include "../gtest/gtest.h"

#include "../../search/probability_distribution.h"

using std::vector;
using std::map;

// Class to fake generation of random numbers
class RandomFake : Random {
    RandomFake() : counter(0) {}

    int genInt(int min, int) override {
        return min;
    }

    // Alternate generated numbers
    double genDouble(double, double) override {
        ++counter;
        if (counter % 2) {
            return 0.2;
        } else {
            return 0.4;
        }
    }

    double genReal() override {
        return 0.0;
    }

    int counter;
};

// Probability distribution only has a single value
TEST(PDTest, testDiracSample) {
    DiscretePD pd;
    pd.assignDiracDelta(5.0);
    // Sampling should always return 5.0
    for (int i = 0; i < 1000; ++i) {
        ASSERT_DOUBLE_EQ(5.0, pd.sample());
    }
    // First and only value is blacklisted, resulting in an assertion error
    vector<int> blacklist{0};
    ASSERT_DEATH(pd.sample(blacklist), "");
}

TEST(PDTest, testDiscretePDSample) {
    DiscretePD pd;
    map<double, double> valueProbPairs = {{1.0, 0.2}, {2.0, 0.2}, {3.0, 0.6}};
    pd.assignDiscrete(valueProbPairs);
    // First random number is 0.2, therefore we should return the first value
    ASSERT_DOUBLE_EQ(1.0, pd.sample());
    // Second random number is 0.4, therefore we should return 3.0
    ASSERT_DOUBLE_EQ(2.0, pd.sample());
}

TEST(PDTest, testDiscretePDSampleBlacklist) {
    DiscretePD pd;
    map<double, double> valueProbPairs = {{1.0, 0.2}, {2.0, 0.2}, {3.0, 0.6}};
    // We blacklist value 2.0, therefore the new distribution is equal to
    // 1.0:0.25/3.0:0.75
    vector<int> blacklist = {1};
    pd.assignDiscrete(valueProbPairs);
    // First random number is 0.2, therefore we should return the first value
    ASSERT_DOUBLE_EQ(1.0, pd.sample(blacklist));
    // Second random number is 0.4, therefore we should return 3.0
    ASSERT_DOUBLE_EQ(3.0, pd.sample(blacklist));
}
