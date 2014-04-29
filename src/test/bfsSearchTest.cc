// Copyright 2014, University of Freiburg
// Arbeitsgruppe für Grundlagen der Künstlichen Intelligenz
// Author: Florian Geißer <geisserf@informatik.uni-freiburg.de>

#include <gtest/gtest.h>
#include "../search/prost_planner.h"
#include "../search/parser.h"
#include "../search/breadth_first_search.h"

using std::string;
using std::map;
using std::vector;

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class bfsSearchTest : public testing::Test {
protected:
    virtual void SetUp() {
        // Parse elevator task
        // TODO: Use the parser instead of a precompiled test instance,
        // so that if the parser changes we don't always have to recompile the
        // elevator file
        string problemFileName = "../test/elevators";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc = "[PROST -se [MC-UCT]]";
        planner = new ProstPlanner(plannerDesc);

        // Initialize other variables
        qValue = 10.0;
        parent = new BFSNode();
        childOne = new BFSNode();
        childTwo = new BFSNode();
        childThree = new BFSNode();
    }

    virtual void teardown() {
        delete planner;
        delete parent;
    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    double qValue;
    BFSNode* parent;
    BFSNode* childOne;
    BFSNode* childTwo;
    BFSNode* childThree;
};

// Tests the initialization of a decision node child
TEST_F(bfsSearchTest, testInitializeDecisionNodeChild) {
    BreadthFirstSearch search;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    search.initializeDecisionNodeChild(parent, 0, 10);
    search.initializeDecisionNodeChild(parent, 1, -10);
    EXPECT_DOUBLE_EQ(400, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(400, parent->children[0]->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(-400, parent->children[1]->getExpectedFutureRewardEstimate());
    EXPECT_EQ(0, parent->getNumberOfVisits());
    EXPECT_EQ(0, parent->children[0]->getNumberOfVisits());
    EXPECT_EQ(0, parent->children[1]->getNumberOfVisits());
}

// Tests the outcome selection
TEST_F(bfsSearchTest, testSelectOutcome) {
    // TODO: Write an actual test for this
}

// Tests backup of a decision node leaf
TEST_F(bfsSearchTest, testBackupDecisionNodeLeaf) {
    BreadthFirstSearch search;
    // Create a node with futReward 0 without accessing private members
    parent->children.push_back(childOne);
    search.initializeDecisionNodeChild(parent, 0, 0);
    BFSNode* node = parent->children[0];
    search.backupDecisionNodeLeaf(node, 10, 20);
    EXPECT_DOUBLE_EQ(20, node->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(30, node->getExpectedRewardEstimate());
    ASSERT_TRUE(node->isSolved());
}

// Tests backup of a decision node
TEST_F(bfsSearchTest, testBackupDecisionNode) {
    BreadthFirstSearch search;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);

    // Initialize children. Note that horizon is 40, thus the expected future
    // reward will be 0, 4, 2
    search.initializeDecisionNodeChild(parent, 0, 0);
    search.initializeDecisionNodeChild(parent, 1, 0.1);
    search.initializeDecisionNodeChild(parent, 2, 0.05);

    delete childOne;
    delete childTwo;
    delete childThree;

    // Assign correct children
    childOne = parent->children[0];
    childTwo = parent->children[1];
    childThree = parent->children[2];

    // Increase future reward of one child and backup parent
    search.backupDecisionNodeLeaf(childOne, 0, 10);
    search.backupDecisionNode(parent, 2, -5);
    EXPECT_DOUBLE_EQ(10, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(12, parent->getExpectedRewardEstimate());
    ASSERT_FALSE(search.backupLock);

    // Increase another child with a lower value, backupLock should get true
    search.backupDecisionNodeLeaf(childTwo, 0, 5);
    search.backupDecisionNode(parent, 2, -1);
    EXPECT_DOUBLE_EQ(10, parent->getExpectedFutureRewardEstimate());
    EXPECT_DOUBLE_EQ(12, parent->getExpectedRewardEstimate());
    ASSERT_TRUE(search.backupLock);
}

// Tests backup of a chance node
TEST_F(bfsSearchTest, testBackupChanceNode) {
    // TODO Test case
}

// Tests action selection for breadth first search
TEST_F(bfsSearchTest, testSelectAction) {
    BreadthFirstSearch search;

    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);


    // TODO: Change the test so that we don't have to exploit private access of
    // visits
    int selectedActionIndex = search.selectAction(parent);
    ASSERT_EQ(0, selectedActionIndex);
    childOne->numberOfVisits++;
    selectedActionIndex = search.selectAction(parent);
    ASSERT_EQ(1, selectedActionIndex);
    childTwo->numberOfVisits++;
    selectedActionIndex = search.selectAction(parent);
    ASSERT_EQ(2, selectedActionIndex);
    childThree->numberOfVisits++;
    selectedActionIndex = search.selectAction(parent);
    ASSERT_EQ(0, selectedActionIndex);

    // TODO: Write a test where not every index of the children is used. Right
    // now the method works for these cases, but the tests should be implemented
    // to preserve maintainability.

    // TODO: Write a test case where nodes get solved and action selection
    // should not select them again
}
