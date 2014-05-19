#include <gtest/gtest.h>
#include "../../rddl_parser/rddl_parser.h"
#include "../../rddl_parser/instantiator.h"
#include "../../rddl_parser/preprocessor.h"
#include "../../rddl_parser/planning_task.h"
#include "../../rddl_parser/evaluatables.h"
using std::string;
using std::vector;
using std::map;
using std::set;
using std::numeric_limits;

// Skill teaching domain includes +,*,-,neg, state, constant, unaries
TEST(preprocessorTest, calculateMinMaxRewardWithoutVectorCaching) {
    // Prepare test setting
    string domainFileName = "../test/parser/skill_teaching_mdp.rddl_prefix"; 
    string problemFileName = "../test/parser/skill_teaching_inst_mdp__9.rddl";
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFileName, problemFileName);
    Instantiator instantiator(task);
    instantiator.instantiate();
    Preprocessor preprocessor(task);
    preprocessor.preprocess();

    double minValue = *(task->rewardCPF->domain.begin());
    double maxValue = *(task->rewardCPF->domain.rbegin());

    ASSERT_NEAR(-19.6723, minValue, 0.0001);
    ASSERT_NEAR(19.6723, maxValue, 0.0001);
}

// Crossing traffic domain includes +,-,neg, state, constant, unaries,
// conjunction
TEST(preprocessorTest, calculateMinMaxRewardWithoutVectorCachingUnary) {
    // Prepare test setting
    string domainfolder = "../../../rddl-domains/ippc2011/rddl_prefix/";
    string domainName = "crossing_traffic";
    string domainFileName = domainfolder + domainName + "_mdp.rddl_prefix";
    string problemFileName = domainfolder + domainName + "_inst_mdp__10.rddl_prefix";
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFileName, problemFileName);
    Instantiator instantiator(task);
    instantiator.instantiate();
    task->rewardCPF->cachingType = "";
    Preprocessor preprocessor(task);
    preprocessor.preprocess();

    double minValue = *(task->rewardCPF->domain.begin());
    double maxValue = *(task->rewardCPF->domain.rbegin());

    ASSERT_NEAR(-1, minValue, 0.0001);
    ASSERT_NEAR(0, maxValue, 0.0001);
}

TEST(preprocessorTest, calculateMinMaxRewardWithoutVectorCachingExists) {
    // Prepare test setting
    string domainfolder = "../../../rddl-domains/ippc2011/rddl_prefix/";
    string domainName = "recon";
    string domainFileName = domainfolder + domainName + "_mdp.rddl_prefix";
    string problemFileName = domainfolder + domainName + "_inst_mdp__1.rddl_prefix";
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFileName, problemFileName);
    Instantiator instantiator(task);
    instantiator.instantiate();
    task->rewardCPF->cachingType = "";
    Preprocessor preprocessor(task);
    preprocessor.preprocess();

    double minValue = *(task->rewardCPF->domain.begin());
    double maxValue = *(task->rewardCPF->domain.rbegin());

    ASSERT_NEAR( -0.7311116, minValue, 0.0001);
    ASSERT_NEAR(0.18377236, maxValue, 0.0001);
}
TEST(preprocessorTest, calculateDomainTest) {
     vector<set<double> > vec;
     ActionState action(0);
     double minRes = numeric_limits<double>::max();
     double maxRes = -numeric_limits<double>::max();
}
