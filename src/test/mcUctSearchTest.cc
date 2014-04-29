// Copyright 2014, University of Freiburg
// Arbeitsgruppe für Grundlagen der Künstlichen Intelligenz
// Author: Florian Geißer <geisserf@informatik.uni-freiburg.de>

#include <gtest/gtest.h>
#include "../search/uct_base.h"
#include "../search/mc_uct_search.h"
#include "../search/prost_planner.h"
#include "../search/parser.h"

using std::string;
using std::map;
using std::vector;

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class mcUctSearchTest : public testing::Test {
protected:
    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
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

        // initialize other variables
        initVisits = 1;
        qValue = 10.0;
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
    double qValue;
};

// tests the initialization of a decicion node child
TEST_F(mcUctSearchTest, testInitializeDecisionNodeChild) {
    MCUCTSearch uctSearch;
    uctSearch.setNumberOfInitialVisits(initVisits);
    MCUCTNode* parent = new MCUCTNode();
    MCUCTNode* child = new MCUCTNode();
    parent->children.push_back(child);
    uctSearch.initializeDecisionNodeChild(parent, 0, qValue);
    EXPECT_DOUBLE_EQ(400, parent->getExpectedFutureRewardEstimate());
    EXPECT_EQ(1, parent->getNumberOfVisits());

    delete parent;
    delete child;
}

// Tests the backup function for decision nodes
TEST_F(mcUctSearchTest, testBackupDecisionNode) {
    MCUCTSearch uctSearch;
    uctSearch.setNumberOfInitialVisits(initVisits);
    MCUCTNode* parent = new MCUCTNode();
    MCUCTNode* child = new MCUCTNode();
    parent->children.push_back(child);
    // sets the future reward to 400 and the visits to 1
    uctSearch.initializeDecisionNodeChild(parent, 0, qValue);
    // performs a backup with immediate and future rewards both 0
    uctSearch.backupDecisionNode(parent, 0, 0);
    EXPECT_EQ(2, parent->getNumberOfVisits());
    EXPECT_DOUBLE_EQ(200, parent->getExpectedRewardEstimate());
    //performs another backup
    uctSearch.backupDecisionNode(parent, 20, 100);
    EXPECT_EQ(3, parent->getNumberOfVisits());
    double res = 520.0 / 3.0;
    EXPECT_DOUBLE_EQ(res, parent->getExpectedRewardEstimate());
    res = 500.0 / 3.0;
    EXPECT_DOUBLE_EQ(res, parent->getExpectedFutureRewardEstimate());
    delete parent;
    delete child;
}
