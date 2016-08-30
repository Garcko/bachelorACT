#include "../gtest/gtest.h"

#include "../../search/parser.h"
#include "../../search/prost_planner.h"
#include "../../search/thts.h"

using std::string;
using std::map;
using std::vector;

class BFSTestSearch : public THTS {
public:
    BFSTestSearch() : THTS("BFSTest") {
        setActionSelection(new BFSActionSelection(this));
        setOutcomeSelection(new UnsolvedMCOutcomeSelection(this));
        setBackupFunction(new PBBackupFunction(this));
    }

    // Wrapper functions to access protected functions
    void wrapInitializeDecisionNodeChild(SearchNode* node,
                                         unsigned int const& actionIndex,
                                         double const& initialQValue) {
        node->children[actionIndex] = getChanceNode(1.0);
        node->children[actionIndex]->futureReward = initialQValue;
        node->children[actionIndex]->numberOfVisits = 1;

        node->numberOfVisits += numberOfInitialVisits;
        node->futureReward = std::max(
            node->futureReward, node->children[actionIndex]->futureReward);
    }

    void wrapBackupDecisionNodeLeaf(SearchNode* node, double const& immReward,
                                    double const& futureReward) {
        node->immediateReward = immReward;
        backupFunction->backupDecisionNodeLeaf(node, futureReward);
    }

    void wrapBackupDecisionNode(SearchNode* node, double const& immReward) {
        node->immediateReward = immReward;
        backupFunction->backupDecisionNode(node);
    }

    void wrapBackupChanceNode(SearchNode* node, double const& futReward) {
        backupFunction->backupChanceNode(node, futReward);
    }

    int wrapSelectAction(SearchNode* node) {
        return actionSelection->selectAction(node);
    }

    void wrapInitStep(State const& _rootState) {
        initStep(_rootState);
    }

    bool getBackupLock() {
        return backupLock;
    }
};

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class BFSSearchTest : public testing::Test {
protected:
    virtual void SetUp() {
        // Parse elevator task
        string problemFileName = "../test/testdomains/elevators_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc =
            "[PROST -se [THTS -act [UCB1] -out [MC] -backup [MC]]]";
        planner = new ProstPlanner(plannerDesc);

        // Initialize other variables
        qValue = 10.0;
        srand(1);
    }

    virtual void teardown() {
        delete planner;
    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string>> stateVariableValues;
    double qValue;
};

// Tests the outcome selection
TEST_F(BFSSearchTest, testBFSSelectOutcome) {
    // TODO: Write an actual test for this
}

// Tests backup of a decision node leaf
TEST_F(BFSSearchTest, testBFSBackupDecisionNodeLeaf) {
    BFSTestSearch search;

    SearchNode* parent = new SearchNode(1.0, 40);
    parent->children.resize(1, nullptr);
    search.wrapInitializeDecisionNodeChild(parent, 0, 0);

    SearchNode* node = parent->children[0];
    search.wrapBackupDecisionNodeLeaf(parent->children[0], 10, 20);
    EXPECT_DOUBLE_EQ(20, node->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(30, node->getExpectedRewardEstimate());
    ASSERT_TRUE(node->solved);

    delete parent;
}

// Tests backup of a decision node
TEST_F(BFSSearchTest, testBFSBackupDecisionNode) {
    BFSTestSearch search;

    SearchNode* parent = new SearchNode(1.0, 40);
    parent->children.resize(3, nullptr);

    // Initialize children. Note that horizon is 40, thus the expected future
    // reward will be 0, 4, 2
    search.wrapInitializeDecisionNodeChild(parent, 0, 0.0);
    search.wrapInitializeDecisionNodeChild(parent, 1, 0.1);
    search.wrapInitializeDecisionNodeChild(parent, 2, 0.05);

    // Increase future reward of one child and backup parent
    search.wrapBackupDecisionNodeLeaf(parent->children[0], 0.0, 10.0);
    search.wrapBackupDecisionNode(parent, 2.0);
    EXPECT_DOUBLE_EQ(10.0, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(12.0, parent->getExpectedRewardEstimate());
    ASSERT_FALSE(search.getBackupLock());

    // Increase another child with a lower value, backupLock should get true
    search.wrapBackupDecisionNodeLeaf(parent->children[1], 0.0, 5.0);
    search.wrapBackupDecisionNode(parent, 2.0);
    EXPECT_DOUBLE_EQ(10.0, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(12.0, parent->getExpectedRewardEstimate());
    ASSERT_TRUE(search.getBackupLock());

    delete parent;
}

// Tests backup of a chance node
TEST_F(BFSSearchTest, testBFSBackupChanceNode) {
    // TODO Test case
}

// Tests action selection for breadth first search
TEST_F(BFSSearchTest, testBFSSelectAction) {
    // TODO: Test case
}
