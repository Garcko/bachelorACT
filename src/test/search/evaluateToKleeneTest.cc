#include <gtest/gtest.h>
#include "../../search/thts.h"
#include "../../search/prost_planner.h"
#include "../../search/parser.h"
#include "../../search/mc_uct_search.h"
using std::string;
using std::vector;
using std::map;
using std::set;
using std::numeric_limits;

class evaluateToKleeneTest : public testing::Test {
protected:

    evaluateToKleeneTest() {
        string domainName = "crossing_traffic";
        string problemFileName = "../../testbed/domains/"+domainName+"_inst_mdp__1";
        Parser parser(problemFileName);
        parser.parseTask(stateVariableIndices, stateVariableValues);

        // Create Prost Planner
        string plannerDesc = "[PROST -se [MC-UCT -i [Uniform]]]";
        planner = new ProstPlanner(plannerDesc);

    }

    // Declares the variables your tests want to use.
    ProstPlanner* planner;
    map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
};

TEST_F(evaluateToKleeneTest, evaluateStateFluent) {
    vector<double> stateVector;
    for (int i = 0; i < State::numberOfDeterministicStateFluents; ++i) {
        stateVector.push_back(0);
    }
    for (int i = 0; i < State::numberOfProbabilisticStateFluents; ++i) {
        stateVector.push_back(0);
    }
    int varIndex = stateVariableIndices["robot-at(x1, y1)"];
    stateVector[varIndex] = 1.0;

    State lowDepthState = State(stateVector, 25);
    State::calcStateFluentHashKeys(lowDepthState);
    State::calcStateHashKey(lowDepthState);
    KleeneState kleene(lowDepthState);
    KleeneState::calcStateHashKey(kleene);
    KleeneState::calcStateFluentHashKeys(kleene);

}
