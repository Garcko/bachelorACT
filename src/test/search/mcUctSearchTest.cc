#include "../gtest/gtest.h"

#include "../../search/prost_planner.h"
#include "../../search/thts.h"
#include "../../search/parser.h"

using std::string;
using std::map;
using std::vector;


class MCUCTTestSearch : public THTS {
public:
    MCUCTTestSearch() :
        THTS("MCUCTTestSearch") {
        setActionSelection(new UCB1ActionSelection(this));
        setOutcomeSelection(new MCOutcomeSelection(this));
        setBackupFunction(new MCBackupFunction(this));
    }

    // Wrapper functions to access protected functions
    void wrapInitializeDecisionNodeChild(SearchNode* node,
                                         unsigned int const& actionIndex,
                                         double const& initialQValue) {
        node->children[actionIndex] = getChanceNode(1.0);
        node->children[actionIndex]->futureReward = initialQValue;
        node->children[actionIndex]->numberOfVisits = 1;

        node->numberOfVisits += numberOfInitialVisits;
        node->futureReward =
            std::max(node->futureReward, node->children[actionIndex]->futureReward);
    }

    void wrapBackupDecisionNode(SearchNode* node,
                                double const& immReward) {
        node->immediateReward = immReward;
        backupFunction->backupDecisionNode(node);
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
        stateVariableIndices.clear();
        stateVariableValues.clear();

        string problemFileName = "../test/testdomains/elevators_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);
        // Create Prost Planner
        string plannerDesc = "[PROST -se [THTS -act [UCB1] -out [MC] -backup [MC] -i [Uniform]]]";
        planner = new ProstPlanner(plannerDesc);
        // initialize other variables
        initVisits = 1;
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
};

// Tests the initialization of a decicion node child
TEST_F(MCUCTSearchTest, testMCUCTInitializeDecisionNodeChild) {
    MCUCTTestSearch uctSearch;
    uctSearch.setNumberOfInitialVisits(initVisits);

    SearchNode* parent = new SearchNode(1.0, 40);
    parent->children.resize(1, nullptr);
    uctSearch.wrapInitializeDecisionNodeChild(parent, 0, 10.0);

    EXPECT_DOUBLE_EQ(10, parent->getExpectedFutureRewardEstimate());
    EXPECT_EQ(1, parent->numberOfVisits);
    delete parent;
}

// Tests the backup function for decision nodes
TEST_F(MCUCTSearchTest, testMCUCTBackupDecisionNode) {
    MCUCTTestSearch uctSearch;
    uctSearch.setNumberOfInitialVisits(initVisits);

    SearchNode* parent = new SearchNode(1.0, 40);
    parent->children.resize(1, nullptr);
    uctSearch.wrapInitializeDecisionNodeChild(parent, 0, 10.0);

    uctSearch.wrapBackupDecisionNode(parent, 0);
    EXPECT_EQ(2, parent->numberOfVisits);
    EXPECT_DOUBLE_EQ(10, parent->getExpectedRewardEstimate());

    // Performs another backup
    uctSearch.wrapBackupDecisionNode(parent, 20);
    EXPECT_EQ(3, parent->numberOfVisits);
    EXPECT_DOUBLE_EQ(30, parent->getExpectedRewardEstimate());
    EXPECT_DOUBLE_EQ(10, parent->getExpectedFutureRewardEstimate());
    delete parent;
}
