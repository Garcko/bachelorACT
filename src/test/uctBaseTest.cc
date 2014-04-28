// Copyright 2014, University of Freiburg
// Arbeitsgruppe für Grundlagen der Künstlichen Intelligenz
// Author: Florian Geißer <geisserf@informatik.uni-freiburg.de>

#include <gtest/gtest.h>
#include "../search/uct_base.h"
#include "../search/mc_uct_search.h"
#include "../search/prost_planner.h"
#include "../search/parser.h"
#include "../search/utils/math_utils.h"

using std::string;
using std::map;
using std::vector;

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class uctBaseTest : public testing::Test {
protected:
    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // Parse elevator task
		// TODO: Use the parser instead of a precompiled test instance
        string problemFileName = "../test/elevators";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc = "[PROST -se [MC-UCT]]";
        planner = new ProstPlanner(plannerDesc);

        // initialize other variables
        initVisits = 1;
        qValue = 10.0;
        parent = new MCUCTNode();
        child = new MCUCTNode();
        childOne = new MCUCTNode();
        childTwo = new MCUCTNode();
        childThree = new MCUCTNode();
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    virtual void TearDown() {
        parent->children.clear();
        delete parent;
        delete planner;
        delete child;
        delete childOne;
        delete childTwo;
        delete childThree;
    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    int initVisits;
    double qValue;
    MCUCTNode* parent;
    MCUCTNode* child;
    MCUCTNode* childOne;
    MCUCTNode* childTwo;
    MCUCTNode* childThree;
};

// tests the UCT selection with a log function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testUCTSelectionWithLOG) {
    MCUCTSearch uctSearch;
    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.selectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // LOG(500)
    ASSERT_NEAR(6.21460, uctSearch.parentVisitPart, 0.0001);
    // VisitPart = magic_constant * sqrt(parentVisitPart/childVisits)
    // = (1000 / 500) * sqrt(6.214/200) = 0.35255
    ASSERT_NEAR(0.35255, uctSearch.visitPart, 0.0001);
    // ASSERT_EQ(x,uctSearch.UCTValue);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.selectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the magicConstant should be 100
    // and therefore the visitPart is 100 * sqrt(6.214/200) = 17.6275
    ASSERT_FLOAT_EQ(100.0, uctSearch.magicConstant);
    ASSERT_NEAR(17.6275, uctSearch.visitPart, 0.0001);
}

// tests the UCT selection with a sqrt function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testUCTSelectionWithSQRT) {
    MCUCTSearch uctSearch;
    uctSearch.setExplorationRateFunction("SQRT");

    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.selectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // SQRT(500)
    ASSERT_NEAR(22.3606, uctSearch.parentVisitPart, 0.0001);
    // visitPart = magic_constant * sqrt(parentVisitPart/childVisits)
    // = (1000 / 500) * sqrt(22.3606/200) = 0.6687
    ASSERT_NEAR(0.6687, uctSearch.visitPart , 0.0001);
    // ASSERT_EQ(x,uctSearch.UCTValue);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.selectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the magicConstant should be 100
    // and therefore the visitPart is 100 * sqrt(22.3606/200) = 33.43
    ASSERT_FLOAT_EQ(100.0, uctSearch.magicConstant);
    ASSERT_NEAR(33.43701, uctSearch.visitPart, 0.0001);
}

// tests the UCT selection with a linear function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testUCTSelectionWithLIN) {
    MCUCTSearch uctSearch;
    uctSearch.setExplorationRateFunction("LIN");

    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.selectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // LIN(500)
    ASSERT_NEAR(500, uctSearch.parentVisitPart, 0.0001);
    // visitPart = magic_constant * sqrt(parentVisitPart/childVisits)
    // = (1000 / 500) * sqrt(500/200) = 3.16227
    ASSERT_NEAR(3.16227, uctSearch.visitPart, 0.0001);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.selectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the magicConstant should be 100
    // and therefore the visitPart is 100 * sqrt(500/200) = 158.11388
    ASSERT_FLOAT_EQ(100.0, uctSearch.magicConstant);
    ASSERT_NEAR(158.11388, uctSearch.visitPart, 0.0001);
}

// tests the UCT selection with the e^sqrt(x) function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testUCTSelectionWithESQRT) {
    MCUCTSearch uctSearch;
    uctSearch.setExplorationRateFunction("E.SQRT");

    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.selectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // log(500)^2
    ASSERT_NEAR(38.62135, uctSearch.parentVisitPart, 0.0001);
    // visitPart = magic_constant * sqrt(parentVisitPart/childVisits)
    // = (1000 / 500) * sqrt(38.62135/200) = 0.87887
    ASSERT_NEAR(0.87887, uctSearch.visitPart, 0.0001);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.selectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the magicConstant should be 100
    // and therefore the visitPart is 100 * sqrt(38.62135/200) = 43.94391
    ASSERT_FLOAT_EQ(100.0, uctSearch.magicConstant);
    ASSERT_NEAR(43.94391, uctSearch.visitPart, 0.0001);
}

// tests different string parameters
TEST_F(uctBaseTest, testValueFromString) {
    MCUCTSearch search;
    string param = "-er";
    string value = "SQRT";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::SQRT, search.explorationRateFunction);
    value = "LOG";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::LOG, search.explorationRateFunction);
    value = "LIN";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::LIN, search.explorationRateFunction);
    value = "E.SQRT";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::LNQUAD, search.explorationRateFunction);

    // before we set uniform root, the value should be false
    EXPECT_FALSE(search.uniformRoot);
    param = "-uniformroot";
    search.setValueFromString(param, value);
    EXPECT_TRUE(search.uniformRoot);

    // before we enable bfs, the value should be false
    EXPECT_FALSE(search.bfs);
    param = "-bfs";
    search.setValueFromString(param, value);
    EXPECT_TRUE(search.bfs);

}

// Tests that uniform selection is applied at the root node.
TEST_F(uctBaseTest, testSelectActionOnRoot) {
    MCUCTSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    ASSERT_EQ(3, parent->children.size());
    parent->numberOfVisits = 3;
    parent->futureReward = 110;
    childOne->numberOfVisits = 1;
    childTwo->numberOfVisits = 1;
    childThree->numberOfVisits = 1;
    childOne->futureReward = 10;
    childTwo->futureReward = 100;
    childThree->futureReward = 0;

    uctSearch.selectAction(parent);
    // Right now we don't have a root node, so we should have selected only the
    // child with the highest reward, i.e. childTwo
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(1, uctSearch.bestActionIndices[0]);

    uctSearch.currentRootNode = parent;
    uctSearch.selectAction(parent);
    // Now we set parent as root node but we still have yet to enable uniform
    // action selection, so this should change nothing
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());

    uctSearch.uniformRoot = true;
    uctSearch.selectAction(parent);
    // After uniform action selection is enabled only the first child should be a
    // potential candidate
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(0, uctSearch.bestActionIndices[0]);
}

// Note that the function just fills the bestActionIndices with every child
// available, which has the effect that in the main selectAction function one
// random child will be selected
TEST_F(uctBaseTest, testSelectRandomAction) {
    MCUCTSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    uctSearch.selectRandomAction(parent);
    ASSERT_EQ(3, uctSearch.bestActionIndices.size());
}

// If a child has not yet been visited it should be selected first.
TEST_F(uctBaseTest, testSelectUnselectedAction) {
    MCUCTSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    ASSERT_EQ(3, parent->children.size());

    childOne->numberOfVisits = 1;
    childTwo->numberOfVisits = 1;
    childOne->futureReward = 10;
    childTwo->futureReward = 100;
    uctSearch.selectUnselectedAction(parent);
    // childThree should be the only action that was selected
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(2, uctSearch.bestActionIndices[0]);

    childTwo->numberOfVisits = 0;
    uctSearch.bestActionIndices.clear();
    uctSearch.selectUnselectedAction(parent);
    // now we have two children that were never visited, so we should have 2
    // best actions
    ASSERT_EQ(2, uctSearch.bestActionIndices.size());
}

// Tests the action selection with the round robin strategy, i.e. breadth first
// search
TEST_F(uctBaseTest, testSelectActionRoundRobin) {
    MCUCTSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    ASSERT_EQ(3, parent->children.size());

    childOne->numberOfVisits = 0;
    childTwo->numberOfVisits = 0;
    childOne->futureReward = 10;
    childTwo->futureReward = 100;
    // initial select
    uctSearch.selectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(0, uctSearch.bestActionIndices[0]);
    childOne->numberOfVisits = 1;
    uctSearch.bestActionIndices.clear();
    // first node was already selected
    uctSearch.selectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(1, uctSearch.bestActionIndices[0]);
    childTwo->numberOfVisits = 1;
    uctSearch.bestActionIndices.clear();
    // second node was already selected
    uctSearch.selectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(2, uctSearch.bestActionIndices[0]);
    childThree->numberOfVisits = 1;
    uctSearch.bestActionIndices.clear();
    // third node was already selected
    uctSearch.selectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.bestActionIndices.size());
    ASSERT_EQ(0, uctSearch.bestActionIndices[0]);
}



