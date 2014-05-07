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

class TestSearch : public MCUCTSearch {
public:
    // wrapper to test protected methods
    int wrapSelectAction(MCUCTNode* node) {
        return selectAction(node);
    }

    void wrapSelectActionBasedOnUCTFormula(MCUCTNode* parent) {
        selectActionBasedOnUCTFormula(parent);
    }

    void wrapSelectActionPerRoundRobin(MCUCTNode* parent) {
        selectActionPerRoundRobin(parent);
    }

    void wrapSelectRandomAction(MCUCTNode* parent) {
        selectRandomAction(parent);
    }

    void wrapSelectUnselectedAction(MCUCTNode* parent) {
        selectUnselectedAction(parent);
    }

    double getParentVisitPart() {
        return parentVisitPart;
    }

    double getVisitPart() {
        return visitPart;
    }

    double getMagicConstant() {
        return magicConstant;
    }

    functionTypes getERFunction() {
        return explorationRateFunction;
    }

    std::vector<int> getBestActionIndices() {
        return bestActionIndices;
    }

    void clearBestActions() {
        bestActionIndices.clear();
    }

    bool getUniformRoot() {
        return uniformRoot;
    }

    void setUniformRoot(bool val) {
        uniformRoot = val;
    }
};

// To use a test fixture, derive from testing::TEST
// A test fixture can set multiple parameter before a test is run
class uctBaseTest : public testing::Test {
protected:
    // Routines called before each test
    virtual void SetUp() {
        // Parse elevator task
        string problemFileName = "../test/elevators";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc = "[PROST -s 1 -se [MC-UCT]]";
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

    // Cleanup after each test
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
TEST_F(uctBaseTest, testMCUCTSelectionWithLOGTWO) {
}

// tests the UCT selection with a log function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testMCUCTSelectionWithLOG) {
    TestSearch uctSearch;
    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // LOG(500)
    ASSERT_NEAR(6.21460, uctSearch.getParentVisitPart(), 0.0001);
    // VisitPart = magic_constant * sqrt(getParentVisitPart()/childVisits)
    // = (1000 / 500) * sqrt(6.214/200) = 0.35255
    ASSERT_NEAR(0.35255, uctSearch.getVisitPart(), 0.0001);
    // ASSERT_EQ(x,uctSearch.UCTValue);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the getMagicConstant() should be 100
    // and therefore the getVisitPart() is 100 * sqrt(6.214/200) = 17.6275
    ASSERT_FLOAT_EQ(100.0, uctSearch.getMagicConstant());
    ASSERT_NEAR(17.6275, uctSearch.getVisitPart(), 0.0001);
}

// tests the UCT selection with a sqrt function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testMCUCTSelectionWithSQRT) {
    TestSearch uctSearch;
    uctSearch.setExplorationRateFunction("SQRT");

    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // SQRT(500)
    ASSERT_NEAR(22.3606, uctSearch.getParentVisitPart(), 0.0001);
    // getVisitPart() = magic_constant * sqrt(getParentVisitPart()/childVisits)
    // = (1000 / 500) * sqrt(22.3606/200) = 0.6687
    ASSERT_NEAR(0.6687, uctSearch.getVisitPart(), 0.0001);
    // ASSERT_EQ(x,uctSearch.UCTValue);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the getMagicConstant() should be 100
    // and therefore the getVisitPart() is 100 * sqrt(22.3606/200) = 33.43
    ASSERT_FLOAT_EQ(100.0, uctSearch.getMagicConstant());
    ASSERT_NEAR(33.43701, uctSearch.getVisitPart(), 0.0001);
}

// tests the UCT selection with a linear function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testMCUCTSelectionWithLIN) {
    TestSearch uctSearch;
    uctSearch.setExplorationRateFunction("LIN");

    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // LIN(500)
    ASSERT_NEAR(500, uctSearch.getParentVisitPart(), 0.0001);
    // getVisitPart() = magic_constant * sqrt(getParentVisitPart()/childVisits)
    // = (1000 / 500) * sqrt(500/200) = 3.16227
    ASSERT_NEAR(3.16227, uctSearch.getVisitPart(), 0.0001);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the getMagicConstant() should be 100
    // and therefore the getVisitPart() is 100 * sqrt(500/200) = 158.11388
    ASSERT_FLOAT_EQ(100.0, uctSearch.getMagicConstant());
    ASSERT_NEAR(158.11388, uctSearch.getVisitPart(), 0.0001);
}

// tests the UCT selection with the e^sqrt(x) function as the exploration-rate
// and McUctSearch as search pattern
// function
TEST_F(uctBaseTest, testMCUCTSelectionWithESQRT) {
    TestSearch uctSearch;
    uctSearch.setExplorationRateFunction("E.SQRT");

    child->futureReward = 50;
    parent->futureReward = 1000;
    parent->numberOfVisits = 500;
    child->numberOfVisits = 200;
    parent->children.push_back(child);
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    ASSERT_EQ(500, parent->getNumberOfVisits());
    ASSERT_EQ(200, child->getNumberOfVisits());
    // log(500)^2
    ASSERT_NEAR(38.62135, uctSearch.getParentVisitPart(), 0.0001);
    // getVisitPart() = magic_constant * sqrt(getParentVisitPart()/childVisits)
    // = (1000 / 500) * sqrt(38.62135/200) = 0.87887
    ASSERT_NEAR(0.87887, uctSearch.getVisitPart(), 0.0001);

    // A test if the future Reward is zero
    parent->futureReward = 0;
    uctSearch.wrapSelectActionBasedOnUCTFormula(parent);
    // since parent.futureReward is zero, the getMagicConstant() should be 100
    // and therefore the getVisitPart() is 100 * sqrt(38.62135/200) = 43.94391
    ASSERT_FLOAT_EQ(100.0, uctSearch.getMagicConstant());
    ASSERT_NEAR(43.94391, uctSearch.getVisitPart(), 0.0001);
}

// tests different string parameters
TEST_F(uctBaseTest, testValueFromString) {
    TestSearch search;
    string param = "-er";
    string value = "SQRT";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::SQRT, search.getERFunction());
    value = "LOG";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::LOG, search.getERFunction());
    value = "LIN";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::LIN, search.getERFunction());
    value = "E.SQRT";
    search.setValueFromString(param, value);
    EXPECT_EQ(UCTBase<MCUCTNode>::LNQUAD, search.getERFunction());

    // before we set uniform root, the value should be false
    EXPECT_FALSE(search.getUniformRoot());
    param = "-uniformroot";
    search.setValueFromString(param, value);
    EXPECT_TRUE(search.getUniformRoot());
}

// Tests that uniform selection is applied at the root node.
TEST_F(uctBaseTest, testSelectActionOnRoot) {
    TestSearch uctSearch;
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

    uctSearch.wrapSelectAction(parent);
    // Right now we don't have a root node, so we should have selected only the
    // child with the highest reward, i.e. childTwo
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(1, uctSearch.getBestActionIndices()[0]);

    uctSearch.setCurrentRootNode(parent);
    uctSearch.wrapSelectAction(parent);
    // Now we set parent as root node but we still have yet to enable uniform
    // action selection, so this should change nothing
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());

    uctSearch.setUniformRoot(true);
    uctSearch.wrapSelectAction(parent);
    // After uniform action selection is enabled only the first child should be a
    // potential candidate
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(0, uctSearch.getBestActionIndices()[0]);
}

// Note that the function just fills the getBestActionIndices() with every child
// available, which has the effect that in the main wrapSelectAction function one
// random child will be selected
TEST_F(uctBaseTest, testSelectRandomAction) {
    TestSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    uctSearch.wrapSelectRandomAction(parent);
    ASSERT_EQ(3, uctSearch.getBestActionIndices().size());
}

// If a child has not yet been visited it should be selected first.
TEST_F(uctBaseTest, testSelectUnselectedAction) {
    TestSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    ASSERT_EQ(3, parent->children.size());

    childOne->numberOfVisits = 1;
    childTwo->numberOfVisits = 1;
    childOne->futureReward = 10;
    childTwo->futureReward = 100;
    uctSearch.wrapSelectUnselectedAction(parent);
    // childThree should be the only action that was selected
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(2, uctSearch.getBestActionIndices()[0]);

    childTwo->numberOfVisits = 0;
    uctSearch.clearBestActions();
    ASSERT_EQ(0, uctSearch.getBestActionIndices().size());
    uctSearch.wrapSelectUnselectedAction(parent);
    // now we have two children that were never visited, so we should have 2
    // best actions
    ASSERT_EQ(2, uctSearch.getBestActionIndices().size());
}

// Tests the action selection with the round robin strategy, i.e. breadth first
// search
TEST_F(uctBaseTest, testSelectActionRoundRobin) {
    TestSearch uctSearch;
    parent->children.push_back(childOne);
    parent->children.push_back(childTwo);
    parent->children.push_back(childThree);
    ASSERT_EQ(3, parent->children.size());

    childOne->numberOfVisits = 0;
    childTwo->numberOfVisits = 0;
    childOne->futureReward = 10;
    childTwo->futureReward = 100;
    // initial select
    uctSearch.wrapSelectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(0, uctSearch.getBestActionIndices()[0]);
    childOne->numberOfVisits = 1;
    uctSearch.clearBestActions();
    // first node was already selected
    uctSearch.wrapSelectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(1, uctSearch.getBestActionIndices()[0]);
    childTwo->numberOfVisits = 1;
    uctSearch.clearBestActions();
    // second node was already selected
    uctSearch.wrapSelectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(2, uctSearch.getBestActionIndices()[0]);
    childThree->numberOfVisits = 1;
    uctSearch.clearBestActions();
    // third node was already selected
    uctSearch.wrapSelectActionPerRoundRobin(parent);
    ASSERT_EQ(1, uctSearch.getBestActionIndices().size());
    ASSERT_EQ(0, uctSearch.getBestActionIndices()[0]);
}
