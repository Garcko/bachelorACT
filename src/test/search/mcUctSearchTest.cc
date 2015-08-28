#include "../gtest/gtest.h"
#include "../../search/uct_base.h"
#include "../../search/mc_uct_search.h"
#include "../../search/prost_planner.h"
#include "../../search/parser.h"

using std::string;
using std::map;
using std::vector;


class MCUCTTestSearch : public MCUCTSearch {
public:

    // Wrapper functions to access protected functions
    void wrapInitializeDecisionNode(THTSSearchNode* node,
                                    std::vector<double> const& initialQValues) {
        // This is mostly a copy of THTS::initializeDecisionNode
        node->futureReward = -std::numeric_limits<double>::max();
        assert(node->children.size() == initialQValues.size());

        for (unsigned int index = 0; index < node->children.size(); ++index) {
            node->children[index]->futureReward =
                (double)node->remainingSteps * initialQValues[index];
            node->children[index]->numberOfVisits = numberOfInitialVisits;

            node->numberOfVisits += numberOfInitialVisits;
            node->futureReward =
                std::max(node->futureReward, node->children[index]->futureReward);
        }
    }

    void wrapBackupDecisionNode(THTSSearchNode* node,
                                double const& immReward,
                                double const& futureReward) {
        node->immediateReward = immReward;
        std::cout << "1" << std::endl;
        backupDecisionNode(node, futureReward);
        std::cout << "3" << std::endl;
    }
};

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class MCUCTSearchTest : public testing::Test {
protected:
    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // Parse elevator task
        string problemFileName = "../test/testdomains/elevators_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);
        // Create Prost Planner
        string plannerDesc = "[PROST -se [MC-UCT]]";
        planner = new ProstPlanner(plannerDesc);

        // initialize other variables
        initVisits = 1;
        qValues.push_back(10.0);
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    virtual void teardown() {
        delete planner;
    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    int initVisits;
    std::vector<double> qValues;
};

// Tests the initialization of a decicion node child
TEST_F(MCUCTSearchTest, testMCUCTInitializeDecisionNodeChild) {
    MCUCTTestSearch uctSearch;
    uctSearch.setNumberOfInitialVisits(initVisits);
    THTSSearchNode* parent = new THTSSearchNode(1.0, 40);
    THTSSearchNode* child = new THTSSearchNode(1.0, 40);
    parent->children.push_back(child);
    uctSearch.wrapInitializeDecisionNode(parent, qValues);
    EXPECT_DOUBLE_EQ(400, parent->getExpectedFutureRewardEstimate());
    EXPECT_EQ(1, parent->getNumberOfVisits());
    delete parent;
    delete child;
}

// Tests the backup function for decision nodes
TEST_F(MCUCTSearchTest, testMCUCTBackupDecisionNode) {
    MCUCTTestSearch uctSearch;
    uctSearch.setNumberOfInitialVisits(initVisits);
     THTSSearchNode* parent = new THTSSearchNode(1.0, 40);
    THTSSearchNode* child = new THTSSearchNode(1.0, 40);
    parent->children.push_back(child);
    // sets the future reward to 400 and the visits to 1
    uctSearch.wrapInitializeDecisionNode(parent, qValues);
    // performs a backup with immediate and future rewards both 0
    uctSearch.wrapBackupDecisionNode(parent, 0, 0);
    EXPECT_EQ(2, parent->getNumberOfVisits());
    EXPECT_DOUBLE_EQ(400, parent->getExpectedRewardEstimate());
    //performs another backup
    uctSearch.wrapBackupDecisionNode(parent, 20, 100);
    EXPECT_EQ(3, parent->getNumberOfVisits());
    double res = 520.0 / 3.0;
    EXPECT_DOUBLE_EQ(res, parent->getExpectedRewardEstimate());
    res = 500.0 / 3.0;
    EXPECT_DOUBLE_EQ(res, parent->getExpectedFutureRewardEstimate());
    delete parent;
    delete child;
}
