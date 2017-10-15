#ifndef THTS_H
#define THTS_H

#include <queue>
#include "search_engine.h"

#include "utils/stopwatch.h"


class ActionSelection;
class OutcomeSelection;
class BackupFunction;
class Initializer;
class RecommendationFunction;

// THTS, Trial-based Heuristic Tree Search, is the implementation of the
// abstract framework described in the ICAPS 2013 paper "Trial-based Heuristic
// Tree Search for Finite Horizon MDPs" (Keller & Helmert). The described
// ingredients (plus the recommendation function that is added in T. Keller's
// PhD dissertation) are implemented in the following five classes (1-5) or as a
// function in this class (6):

// 1. ActionSelection

// 2. Outcome Selection

// 3. BackupFunction

// 4. Initializer

// 5. RecommendationFunction

// 6. continueTrial()

// Add ingredients by deriving from the corresponding class.

struct SearchNode {
    SearchNode(double const& _prob, int const& _stepsToGo)
        : children(),
          immediateReward(0.0),
          prob(_prob),
          stepsToGo(_stepsToGo),
          futureReward(-std::numeric_limits<double>::max()),
          numberOfVisits(0),
          initialized(false),
          solved(false),
          isChanceNode(false),
          isActionNode(false),
          equivalenceClassPos(-1){}


    ~SearchNode() {
        for (unsigned int i = 0; i < children.size(); ++i) {
            if (children[i]) {
                delete children[i];
            }
        }
    }

    void reset(double const& _prob, int const& _stepsToGo) {
        children.clear();
        immediateReward = 0.0;
        prob = _prob;
        stepsToGo = _stepsToGo;
        futureReward = -std::numeric_limits<double>::max();
        numberOfVisits = 0;
        initialized = false;
        solved = false;
		isChanceNode = false;
        isActionNode = false;
        equivalenceClassPos=-1;//empty
    }

    double getExpectedRewardEstimate() const {
        if (equivalenceClassPos == -1) {
            return immediateReward + futureReward;
        } else {
            //hier durchschnitt nehmen aus q value aus beiden vectoren
           //return immediateReward + futureReward;
         //   std::cout << qvalueMean.size() << " / " << equivalenceClassPos << std::endl;
            assert(qvalueMean.size() > equivalenceClassPos);
            assert(equivalenceClassPos >= 0);
            return qvalueMean[equivalenceClassPos];
        }
    }

    double getExpectedFutureRewardEstimate() const {
        return futureReward;
    }

    void print(std::ostream& out, std::string indent = "") const {
        out << indent << "stepsToGO: " << stepsToGo
            << " is ChanceNode: "<< isChanceNode << std::endl;
        if (solved) {
            out << indent << "SOLVED with: " << getExpectedRewardEstimate()
                << " (in " << numberOfVisits << " real visits)" << std::endl;
        } else {
            out << indent << getExpectedRewardEstimate() << " (in "
                << numberOfVisits << " real visits)" << std::endl;
        }
    }
    bool isALeafNode() {
        for (SearchNode* child : children) {
            if (child&&!child->children.empty()) {
                return false;
            }
        }
        return true;
    }
    // pr the probability of the small chancenode sub-tree
    void collectAllDecisionNodeSuccessor(std::vector<std::pair<SearchNode*,double>> &result,double pr=1.0){
        if(!isChanceNode){
            result.push_back(std::make_pair(this,pr));
        }else{
          for(SearchNode* child: children)   {
              if(child){
                  child->collectAllDecisionNodeSuccessor(result,child->prob*pr);
              }
          }
        }

    }

    static std::vector<double> qvalueMean;

    std::vector<SearchNode*> children;

    double immediateReward;
    double prob;
    int stepsToGo;

    double futureReward;
    int numberOfVisits;



    // This is used in two ways: in decision nodes, it is true if all children
    // are initialized; and in chance nodes that represent an action (i.e., in
    // children of decision nodes), it is true if an initial value has been
    // assigned to the node.
    bool initialized;

    // A node is solved if futureReward is equal to the true future reward
    bool solved;

    /* new  */
    bool isChanceNode;	//to different between Chance and Decision Node

    // An action node is a chance node whose parent is a decision node
    bool isActionNode;

    //number of the equivalenzclass
    int equivalenceClassPos;


};


class THTS : public ProbabilisticSearchEngine {

public:
    enum TerminationMethod {
        TIME,                     // stop after timeout sec
        NUMBER_OF_TRIALS,         // stop after maxNumberOfTrials trials
        TIME_AND_NUMBER_OF_TRIALS // stop after timeout sec or maxNumberOfTrials
                                  // trials, whichever comes first
    };

    THTS(std::string _name);

    // Set parameters from command line
    bool setValueFromString(std::string& param, std::string& value) override;

    //  This is called when caching is disabled because memory becomes sparse
    void disableCaching() override;

    // Learns parameter values from a random training set
    void learn() override;

    // Start the search engine as main search engine
    void estimateBestActions(State const& _rootState,
                             std::vector<int>& bestActions) override;

    // Start the search engine to estimate the Q-value of a single action
    void estimateQValue(State const& /*state*/, int /*actionIndex*/,
                        double& /*qValue*/) override {
        assert(false);
    }

    // Start the search engine to estimate the Q-values of all applicable
    // actions
    void estimateQValues(State const& /*state*/,
                         std::vector<int> const& /*actionsToExpand*/,
                         std::vector<double>& /*qValues*/) override {
        assert(false);
    }

    // Parameter setter
    void setActionSelection(ActionSelection* _actionSelection);
    void setOutcomeSelection(OutcomeSelection* _outcomeSelection);
    void setBackupFunction(BackupFunction* _backupFunction);
    void setInitializer(Initializer* _initializer);
    void setRecommendationFunction(
        RecommendationFunction* _recommendationFunction);

    void setMaxSearchDepth(int _maxSearchDepth);
    void setTerminationMethod(THTS::TerminationMethod _terminationMethod) {
        terminationMethod = _terminationMethod;
    }

    void setMaxNumberOfTrials(int _maxNumberOfTrials) {
        maxNumberOfTrials = _maxNumberOfTrials;
    }

    void setNumberOfNewDecisionNodesPerTrial(
        int _numberOfNewDecisionNodesPerTrial) {
        numberOfNewDecisionNodesPerTrial = _numberOfNewDecisionNodesPerTrial;
    }

    void setMaxNumberOfNodes(int _maxNumberOfNodes) {
        maxNumberOfNodes = _maxNumberOfNodes;
        // Resize the node pool and give it a "safety net" of 20000 nodes (this
        // is because the termination criterion is checked only at the root and
        // not in the middle of a trial)
        nodePool.resize(maxNumberOfNodes + 20000, nullptr);
    }
    //compare method for SearchNode used in the priority queue
    struct CompareSearchNodeDepth {
        bool operator()(SearchNode*  lhs, SearchNode*  rhs) const {
            if(lhs->stepsToGo != rhs->stepsToGo){ // not same level
                return lhs->stepsToGo < rhs->stepsToGo;
            }
            //same level , compare Node type
            //return rhs->isChanceNode;

            return lhs->isChanceNode;
        }

    };



    // Methods to create search nodes
    SearchNode* createRootNode();
    SearchNode* createDecisionNode(double const& _prob);
    SearchNode* createChanceNode(double const& _prob,  bool isActionNode);



    // Methods that return certain nodes of the explicated tree
    SearchNode const* getCurrentRootNode() const {
        return currentRootNode;
    }

    SearchNode const* getTipNodeOfTrial() const {
        return tipNodeOfTrial;
    }

    // Print
    void printStats(std::ostream& out, bool const& printRoundStats,
                    std::string indent = "") const override;


    /*new */


private:
    // Main search functions
    void visitDecisionNode(SearchNode* node);
    void visitChanceNode(SearchNode* node);
    void visitDummyChanceNode(SearchNode* node);

    // Initialization of different search phases
    void initRound();
    void initStep(State const& _rootState);
    void initTrial();
    void initTrialStep();

    // Trial length determinization
    bool continueTrial(SearchNode* /*node*/) {
        return initializedDecisionNodes < numberOfNewDecisionNodesPerTrial;
    }

    // Determines if the current state has been solved before or can be solved
    // now
    bool currentStateIsSolved(SearchNode* node);

    // If the root state is a reward lock or has only one reasonable action,
    // noop or the only reasonable action is returned
    int getUniquePolicy();

    // Determine if another trial is performed
    bool moreTrials();

    // Ingredients that are implemented externally
    ActionSelection* actionSelection;
    OutcomeSelection* outcomeSelection;
    BackupFunction* backupFunction;
    Initializer* initializer;
    RecommendationFunction* recommendationFunction;

    // Search nodes used in trials
    SearchNode* currentRootNode;
    SearchNode* chosenOutcome;

    // The tip node of a trial is the first node that is encountered that
    // requires initialization of a child
    SearchNode* tipNodeOfTrial;

    // The path of states that is traversed in a trial (such that states[x] is
    // the state that is visited with x steps-to-go)
    std::vector<PDState> states;

    // Indices that allow simple access to the current state, action etc.
    int stepsToGoInCurrentState;
    int stepsToGoInNextState;
    int appliedActionIndex;

    // The accumulated reward that has been achieved in the current trial (the
    // rewards are accumulated in reverse order during the backup phase, such
    // that it reflects the future reward in each node when it is backed up)
    double trialReward;

    // Counter for the number of trials
    int currentTrial;

    // Max search depth for the current step
    int maxSearchDepthForThisStep;

    // Variable used to navigate through chance node layers
    int chanceNodeVarIndex;

    // Index of the last variable with non-deterministic outcome in the current
    // transition
    int lastProbabilisticVarIndex;

    // Counter for the number of decision nodes that have been initialized in
    // the current trial
    int initializedDecisionNodes;

    // Memory management (nodePool)
    int lastUsedNodePoolIndex;
    std::vector<SearchNode*> nodePool;

    // The stopwatch used for timeout check
    Stopwatch stopwatch;

    // Parameter
    THTS::TerminationMethod terminationMethod;
    int maxNumberOfTrials;
    int numberOfNewDecisionNodesPerTrial;
    int maxNumberOfNodes;

    // Statistics
    int numberOfRuns;
    int cacheHits;
    int accumulatedNumberOfStepsToGoInFirstSolvedRootState;
    bool firstSolvedFound;
    int accumulatedNumberOfTrialsInRootState;
    int accumulatedNumberOfSearchNodesInRootState;

    // Tests which access private members
    friend class THTSTest;
    friend class BFSTestSearch;
    friend class MCUCTTestSearch;
    friend class UCTBaseTestSearch;

    //PriorityQueue
    std::multiset <SearchNode*,CompareSearchNodeDepth> pq;


    std::vector<std::set<SearchNode*>> vectorEquivalenceClass;
    std::vector< std::map<int,double>> vectorChildrenOnLevel;


    std::map<int,double> tempMap;   //current searchnode children
    std::map<int,double> currentChildrenMap;

    int currentLevel;
    int numberOfEQclasses;
    int leaveEQCLass;
    int currentLeaveLevel;

    bool leaveisChanceNode;
    bool currentIsChanceNode;

    bool overlappingEQclass=false; // if its true  the EQ class can overlap over multiple level ,else it is levelinternal
    //default false; n

    int childEQ;
    bool isSameEQClass;
    std::vector<double> qvalueSum;
    std::vector<double>qvalueNumbersOfEQClasses;


    std::vector<std::pair<SearchNode*,double>> specialChildren;

    double timestep; // after how many trials the  EQ classes are generated
    double lasttime;

    void generateEquivalenceClass();
    std::map<int,double> makeChildrenOnLevel(SearchNode*);

    void makeQmean();

    std::chrono::steady_clock::time_point time_before;  //time before generateEQ class
    double time_interval;    // how long the generateEQ class take times , this is subtracted from the current time
    // therefore the generate EQ class time has now no impact on the timeout !!

};

#endif
