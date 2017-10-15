#include "thts.h"

#include "action_selection.h"
#include "backup_function.h"
#include "initializer.h"
#include "outcome_selection.h"
#include "recommendation_function.h"

#include "utils/system_utils.h"

std::vector<double> SearchNode::qvalueMean;

/******************************************************************
                     Search Engine Creation
******************************************************************/

THTS::THTS(std::string _name)
        : ProbabilisticSearchEngine(_name),
          actionSelection(nullptr),
          outcomeSelection(nullptr),
          backupFunction(nullptr),
          initializer(nullptr),
          recommendationFunction(nullptr),
          currentRootNode(nullptr),
          chosenOutcome(nullptr),
          tipNodeOfTrial(nullptr),
          states(SearchEngine::horizon + 1),
          stepsToGoInCurrentState(SearchEngine::horizon),
          stepsToGoInNextState(SearchEngine::horizon - 1),
          appliedActionIndex(-1),
          trialReward(0.0),
          currentTrial(0),
          initializedDecisionNodes(0),
          lastUsedNodePoolIndex(0),
          terminationMethod(THTS::TIME),
          maxNumberOfTrials(0),
          numberOfNewDecisionNodesPerTrial(1),
          numberOfRuns(0),
          cacheHits(0),
          accumulatedNumberOfStepsToGoInFirstSolvedRootState(0),
          firstSolvedFound(false),
          accumulatedNumberOfTrialsInRootState(0),
          accumulatedNumberOfSearchNodesInRootState(0),
          timestep(0.01),
          lasttime(0.0) {
    setMaxNumberOfNodes(24000000);
    setTimeout(1.0);
    setRecommendationFunction(new ExpectedBestArmRecommendation(this));
}

bool THTS::setValueFromString(std::string &param, std::string &value) {
    // Check if this parameter encodes an ingredient
    if (param == "-act") {
        setActionSelection(ActionSelection::fromString(value, this));

        return true;
    } else if (param == "-out") {
        setOutcomeSelection(OutcomeSelection::fromString(value, this));

        return true;
    } else if (param == "-backup") {
        setBackupFunction(BackupFunction::fromString(value, this));

        return true;
    } else if (param == "-init") {
        setInitializer(Initializer::fromString(value, this));
        return true;
    } else if (param == "-rec") {
        setRecommendationFunction(
                RecommendationFunction::fromString(value, this));
        return true;
    }

    if (param == "-T") {
        if (value == "TIME") {
            setTerminationMethod(THTS::TIME);
            return true;
        } else if (value == "TRIALS") {
            setTerminationMethod(THTS::NUMBER_OF_TRIALS);
            return true;
        } else if (value == "TIME_AND_TRIALS") {
            setTerminationMethod(THTS::TIME_AND_NUMBER_OF_TRIALS);
            return true;
        } else {
            return false;
        }
    } else if (param == "-r") {
        setMaxNumberOfTrials(atoi(value.c_str()));
        return true;
    } else if (param == "-ndn") {
        if (value == "H") {
            setNumberOfNewDecisionNodesPerTrial(SearchEngine::horizon);
        } else {
            setNumberOfNewDecisionNodesPerTrial(atoi(value.c_str()));
        }
        return true;
    } else if (param == "-node-limit") {
        setMaxNumberOfNodes(atoi(value.c_str()));
        return true;
    } else if (param == "-uf") {
        timestep = atof(value.c_str());
        return true;
    }

    return SearchEngine::setValueFromString(param, value);
}

void THTS::setActionSelection(ActionSelection *_actionSelection) {
    if (actionSelection) {
        delete actionSelection;
    }
    actionSelection = _actionSelection;
}

void THTS::setOutcomeSelection(OutcomeSelection *_outcomeSelection) {
    if (outcomeSelection) {
        delete outcomeSelection;
    }
    outcomeSelection = _outcomeSelection;
}

void THTS::setBackupFunction(BackupFunction *_backupFunction) {
    if (backupFunction) {
        delete backupFunction;
    }
    backupFunction = _backupFunction;
}

void THTS::setInitializer(Initializer *_initializer) {
    if (initializer) {
        delete initializer;
    }
    initializer = _initializer;
}

void THTS::setRecommendationFunction(
        RecommendationFunction *_recommendationFunction) {
    if (recommendationFunction) {
        delete recommendationFunction;
    }
    recommendationFunction = _recommendationFunction;
}

/******************************************************************
                 Search Engine Administration
******************************************************************/

void THTS::disableCaching() {
    actionSelection->disableCaching();
    outcomeSelection->disableCaching();
    backupFunction->disableCaching();
    initializer->disableCaching();
    recommendationFunction->disableCaching();
    SearchEngine::disableCaching();
}

void THTS::learn() {
    // All ingredients must have been specified
    if (!actionSelection || !outcomeSelection || !backupFunction ||
        !initializer || !recommendationFunction) {
        SystemUtils::abort(
                "Action selection, outcome selection, backup "
                        "function, initializer, and recommendation function "
                        "must be defined in a THTS search engine!");
    }

    std::cout << name << ": learning..." << std::endl;
    actionSelection->learn();
    outcomeSelection->learn();
    backupFunction->learn();
    initializer->learn();
    recommendationFunction->learn();
    std::cout << name << ": ...finished" << std::endl;
}

/******************************************************************
                 Initialization of search phases
******************************************************************/

void THTS::initRound() {
    std::cout << name << "t init round"<<std::endl;
    pq.clear();
    firstSolvedFound = false;

    actionSelection->initRound();
    outcomeSelection->initRound();
    backupFunction->initRound();
    initializer->initRound();
    recommendationFunction->initRound();
}

void THTS::initStep(State const &_rootState) {
    pq.clear(); //
    PDState rootState(_rootState);
    // Adjust maximal search depth and set root state
    if (rootState.stepsToGo() > maxSearchDepth) {
        maxSearchDepthForThisStep = maxSearchDepth;
        states[maxSearchDepthForThisStep].setTo(rootState);
        states[maxSearchDepthForThisStep].stepsToGo() =
                maxSearchDepthForThisStep;
    } else {
        maxSearchDepthForThisStep = rootState.stepsToGo();
        states[maxSearchDepthForThisStep].setTo(rootState);
    }
    assert(states[maxSearchDepthForThisStep].stepsToGo() ==
           maxSearchDepthForThisStep);

    stepsToGoInCurrentState = maxSearchDepthForThisStep;
    stepsToGoInNextState = maxSearchDepthForThisStep - 1;
    states[stepsToGoInNextState].reset(stepsToGoInNextState);

    // Reset step dependent counter
    currentTrial = 0;
    cacheHits = 0;

    // Reset search nodes and create root node
    currentRootNode = createRootNode();

    std::cout << name << ": Maximal search depth set to "
              << maxSearchDepthForThisStep << std::endl
              << std::endl;
}

inline void THTS::initTrial() {
    // Reset states and steps-to-go counter
    stepsToGoInCurrentState = maxSearchDepthForThisStep;
    stepsToGoInNextState = maxSearchDepthForThisStep - 1;
    states[stepsToGoInNextState].reset(stepsToGoInNextState);

    // Reset trial dependent variables
    initializedDecisionNodes = 0;
    trialReward = 0.0;
    tipNodeOfTrial = nullptr;

    // Init trial in ingredients
    actionSelection->initTrial();
    outcomeSelection->initTrial();
    backupFunction->initTrial();
    initializer->initTrial();
}

inline void THTS::initTrialStep() {
    --stepsToGoInCurrentState;
    --stepsToGoInNextState;
    states[stepsToGoInNextState].reset(stepsToGoInNextState);
}

/******************************************************************
                       Main Search Functions
******************************************************************/

void THTS::estimateBestActions(State const &_rootState,
                               std::vector<int> &bestActions) {
    assert(bestActions.empty());

    stopwatch.reset();
    lasttime = 0.0;
    //time_interval=0.0;

    // Init round (if this is the first call in a round)
    if (_rootState.stepsToGo() == SearchEngine::horizon) {
        initRound();
    }

    // Init step (this function is currently only called once per step) TODO:
    // maybe we should call initRound, initStep and printStats from "outside"
    // such that we can also use this as a heuristic without generating too much
    // output
    initStep(_rootState);

    // Check if there is an obviously optimal policy (as, e.g., in the last step
    // or in a reward lock)
    int uniquePolicyOpIndex = getUniquePolicy();
    if (uniquePolicyOpIndex != -1) {
        std::cout << "Returning unique policy: ";
        SearchEngine::actionStates[uniquePolicyOpIndex].printCompact(std::cout);
        std::cout << std::endl << std::endl;
        bestActions.push_back(uniquePolicyOpIndex);
        currentRootNode = nullptr;
        printStats(std::cout, (_rootState.stepsToGo() == 1));
        return;
    }

    // Start the main loop that starts trials until some termination criterion
    // is fullfilled
    while (moreTrials()) {
        // std::cout <<
        // "---------------------------------------------------------" <<
        // std::endl;
        //  std::cout << "TRIAL " << (currentTrial+1) << std::endl;
        // std::cout <<
        // "---------------------------------------------------------" <<
        // std::endl;
        visitDecisionNode(currentRootNode);
        //  std::cout << "visited decision node  " <<std::endl;
        ++currentTrial;


        /*
       for(unsigned int i = 0; i < currentRootNode->children.size(); ++i) {
            if(currentRootNode->children[i]) {
                SearchEngine::actionStates[i].print(std::cout);
                std::cout << std::endl;
                currentRootNode->children[i]->print(std::cout, "  ");
            }
        }
*/


        if (stopwatch() - lasttime >= timestep) {  //parameter alle modul zeit

           // std::cout << "starting  " << std::endl;

            //Stopwatch t;
            stopwatch.saveTime();
            generateEquivalenceClass();
            stopwatch.continueTime();
            lasttime = stopwatch();
            // time_interval+=t();
            std::cout << "/" << lasttime << " / " << std::endl;
            // time2=std::chrono::steady_clock::now();




            //  std::cout << "finished generate EQ class" << std::endl;

            // assert(currentTrial == 100);
        }

    }
    recommendationFunction->recommend(currentRootNode, bestActions);
    assert(!bestActions.empty());

    // Update statistics
    ++numberOfRuns;

    if (currentRootNode->solved && !firstSolvedFound) {
        // TODO: This is the first root state that was solved, so everything
        // that could happen in the future is also solved. We should (at least
        // in this case) make sure that we keep the tree and simply follow the
        // optimal policy.
        firstSolvedFound = true;
        accumulatedNumberOfStepsToGoInFirstSolvedRootState +=
                _rootState.stepsToGo();
    }

    if (_rootState.stepsToGo() == SearchEngine::horizon) {
        accumulatedNumberOfTrialsInRootState += currentTrial;
        accumulatedNumberOfSearchNodesInRootState += lastUsedNodePoolIndex;
    }

    // Print statistics
    std::cout << "Search time: " << stopwatch << std::endl;
    printStats(std::cout, (_rootState.stepsToGo() == 1));
}

bool THTS::moreTrials() {
    // Check memory constraints and solvedness
    if (currentRootNode->solved ||
        (lastUsedNodePoolIndex >= maxNumberOfNodes)) {
        return false;
    }

    if (currentTrial == 0) {
        return true;
    }

    // Check selected termination criterion
    switch (terminationMethod) {
        case THTS::TIME:
            if (MathUtils::doubleIsGreater(stopwatch() - time_interval, timeout)) {
                return false;
            }
            break;
        case THTS::NUMBER_OF_TRIALS:
            if (currentTrial == maxNumberOfTrials) {
                return false;
            }
            break;
        case THTS::TIME_AND_NUMBER_OF_TRIALS:
            if (MathUtils::doubleIsGreater(stopwatch() - time_interval, timeout) ||
                (currentTrial == maxNumberOfTrials)) {
                return false;
            }
            break;
    }

    return true;
}

void THTS::visitDecisionNode(SearchNode *node) {
    //  std::cout << "t visit decison node  " <<std::endl;
    if (node == currentRootNode) {
        initTrial();
    } else {

        // Continue trial (i.e., set next state to be the current)
        initTrialStep();

        // Check if there is a "special" reason to stop this trial (currently,
        // this is the case if the state value of the current state is cached,
        // if it is a reward lock or if there is only one step left).
        if (currentStateIsSolved(node)) {
            if (!tipNodeOfTrial) {
                tipNodeOfTrial = node;
            }
            //      std::cout << "t special case  " <<std::endl;
            pq.insert(node);
            return;
        }
    }
    //  std::cout << "t nothing special " <<std::endl;
    // Initialize node if necessary
    if (!node->initialized) {
        if (!tipNodeOfTrial) {
            tipNodeOfTrial = node;
        }

        initializer->initialize(node, states[stepsToGoInCurrentState]);
        //add node+children  to the multiset
        pq.insert(node);
        //  std::cout << "parent level: "<<node->stepsToGo << " is a ChanceNode  " <<node->isChanceNode << "        and isleaf  " <<node->isALeafNode()<<std::endl;

        // add the chanceNode children of the decision node to the multiset if they exist
        for (SearchNode *child : node->children) {
            if (child) {
                pq.insert(child);
                //    std::cout << "level: "<<child->stepsToGo << " is a ChanceNode  " <<child->isChanceNode << "         and isleaf  " <<child->isALeafNode()<<std::endl;
                // std::cout << "with children size"<<child->children.size()  <<std::endl;
            }
        }


        if (node != currentRootNode) {
            ++initializedDecisionNodes;
        }
    }
    // std::cout << "t not initialized " <<std::endl;

    // std::cout << std::endl << std::endl << "Current state is: " << std::endl;
    // states[stepsToGoInCurrentState].printCompact(std::cout);
    //std::cout << "Reward is " << node->immediateReward << std::endl;

    // std::cout << "t before continue trial  " <<std::endl;
    // Determine if we continue with this trial
    if (continueTrial(node)) {
        //    std::cout << "t in continue trial " <<std::endl;
        // Select the action that is simulated
        appliedActionIndex = actionSelection->selectAction(node);
        //   std::cout << "t after  selectAction " <<std::endl;
        assert(node->children[appliedActionIndex]);
        assert(!node->children[appliedActionIndex]->solved);

        // std::cout << "Chosen action is: ";
        // SearchEngine::actionStates[appliedActionIndex].printCompact(std::cout);
        // std::cout << std::endl;

        // Sample successor state
        //std::cout << "t before calc " <<std::endl;
        calcSuccessorState(states[stepsToGoInCurrentState], appliedActionIndex,
                           states[stepsToGoInNextState]);
        //std::cout << "t after calc " <<std::endl;

        // std::cout << "Sampled PDState is " << std::endl;
        // states[stepsToGoInNextState].printPDStateCompact(std::cout);
        // std::cout << std::endl;

        lastProbabilisticVarIndex = -1;
        for (unsigned int i = 0; i < State::numberOfProbabilisticStateFluents;
             ++i) {
            if (states[stepsToGoInNextState]
                    .probabilisticStateFluentAsPD(i)
                    .isDeterministic()) {
                states[stepsToGoInNextState].probabilisticStateFluent(i) =
                        states[stepsToGoInNextState]
                                .probabilisticStateFluentAsPD(i)
                                .values[0];
            } else {
                lastProbabilisticVarIndex = i;
            }
        }

        // Start outcome selection with the first probabilistic variable
        chanceNodeVarIndex = 0;
        // std::cout << "t before visitng NODES " <<std::endl;
        // Continue trial with chance nodes
        if (lastProbabilisticVarIndex < 0) {
            visitDummyChanceNode(node->children[appliedActionIndex]);
        } else {
            visitChanceNode(node->children[appliedActionIndex]);
        }

        // std::cout << "t before backup " <<std::endl;
        // Backup this node
        backupFunction->backupDecisionNode(node);
        trialReward += node->immediateReward;

        // std::cout << "t after backup " <<std::endl;
        // If the backup function labeled the node as solved, we store the
        // result for the associated state in case we encounter it somewhere
        // else in the tree in the future
        if (node->solved) {
            if (cachingEnabled &&
                ProbabilisticSearchEngine::stateValueCache.find(
                        states[node->stepsToGo]) ==
                ProbabilisticSearchEngine::stateValueCache.end()) {
                ProbabilisticSearchEngine::stateValueCache
                [states[node->stepsToGo]] =
                        node->getExpectedFutureRewardEstimate();
            }
        }
    } else {
        // The trial is finished
        //  std::cout << "t trial is finished " <<std::endl;
        trialReward = node->getExpectedRewardEstimate();
    }
}

bool THTS::currentStateIsSolved(SearchNode *node) {
    if (stepsToGoInCurrentState == 1) {
        // This node is a leaf (there is still a last decision, though, but that
        // is taken care of by calcOptimalFinalReward)

        calcOptimalFinalReward(states[1], trialReward);
        backupFunction->backupDecisionNodeLeaf(node, trialReward);
        trialReward += node->immediateReward;

        return true;
    } else if (ProbabilisticSearchEngine::stateValueCache.find(
            states[stepsToGoInCurrentState]) !=
               ProbabilisticSearchEngine::stateValueCache.end()) {
        // This state has already been solved before
        trialReward = ProbabilisticSearchEngine::stateValueCache
        [states[stepsToGoInCurrentState]];
        backupFunction->backupDecisionNodeLeaf(node, trialReward);
        trialReward += node->immediateReward;

        ++cacheHits;
        return true;
    } else if (node->children.empty() &&
               isARewardLock(states[stepsToGoInCurrentState])) {
        // This state is a reward lock, i.e. a goal or a state that is such that
        // no matter which action is applied we'll always get the same reward

        calcReward(states[stepsToGoInCurrentState], 0, trialReward);
        trialReward *= stepsToGoInCurrentState;
        backupFunction->backupDecisionNodeLeaf(node, trialReward);
        trialReward += node->immediateReward;

        if (cachingEnabled) {
            assert(ProbabilisticSearchEngine::stateValueCache.find(
                    states[stepsToGoInCurrentState]) ==
                   ProbabilisticSearchEngine::stateValueCache.end());
            ProbabilisticSearchEngine::stateValueCache
            [states[stepsToGoInCurrentState]] =
                    node->getExpectedFutureRewardEstimate();
        }
        return true;
    }
    return false;
}

void THTS::visitChanceNode(SearchNode *node) {
    while (states[stepsToGoInNextState]
            .probabilisticStateFluentAsPD(chanceNodeVarIndex)
            .isDeterministic()) {
        ++chanceNodeVarIndex;
    }

    chosenOutcome = outcomeSelection->selectOutcome(
            node, states[stepsToGoInNextState], chanceNodeVarIndex,
            lastProbabilisticVarIndex);

    if (chanceNodeVarIndex == lastProbabilisticVarIndex) {
        State::calcStateFluentHashKeys(states[stepsToGoInNextState]);
        State::calcStateHashKey(states[stepsToGoInNextState]);

        visitDecisionNode(chosenOutcome);
    } else {
        ++chanceNodeVarIndex;
        visitChanceNode(chosenOutcome);
    }
    backupFunction->backupChanceNode(node, trialReward);
}

void THTS::visitDummyChanceNode(SearchNode *node) {
    State::calcStateFluentHashKeys(states[stepsToGoInNextState]);
    State::calcStateHashKey(states[stepsToGoInNextState]);

    if (node->children.empty()) {
        node->children.resize(1, nullptr);
        node->children[0] = createDecisionNode(1.0);
    }
    assert(node->children.size() == 1);

    visitDecisionNode(node->children[0]);
    backupFunction->backupChanceNode(node, trialReward);
}

/******************************************************************
                      Root State Analysis
******************************************************************/

int THTS::getUniquePolicy() {
    if (stepsToGoInCurrentState == 1) {
        std::cout << "Returning the optimal last action!" << std::endl;
        return getOptimalFinalActionIndex(states[1]);
    }

    std::vector<int> actionsToExpand =
            getApplicableActions(states[stepsToGoInCurrentState]);

    if (isARewardLock(states[stepsToGoInCurrentState])) {
        std::cout << "Current root state is a reward lock state!" << std::endl;
        states[stepsToGoInCurrentState].print(std::cout);
        for (unsigned int i = 0; i < actionsToExpand.size(); ++i) {
            if (actionsToExpand[i] == i) {
                return i;
            }
        }

        assert(false);
    }

    std::vector<int> applicableActionIndices =
            getIndicesOfApplicableActions(states[stepsToGoInCurrentState]);
    assert(!applicableActionIndices.empty());

    if (applicableActionIndices.size() == 1) {
        std::cout << "Only one reasonable action in current root state!"
                  << std::endl;
        return applicableActionIndices[0];
    }

    // There is more than one applicable action
    return -1;
}

/******************************************************************
                        Memory management
******************************************************************/

SearchNode *THTS::createRootNode() {
    for (SearchNode *node : nodePool) {
        if (node) {
            if (!node->children.empty()) {
                std::vector<SearchNode *> tmp;
                node->children.swap(tmp);
            }
        } else {
            break;
        }
    }

    SearchNode *res = nodePool[0];

    if (res) {
        res->reset(1.0, stepsToGoInCurrentState);
    } else {
        res = new SearchNode(1.0, stepsToGoInCurrentState);
        nodePool[0] = res;
    }
    res->immediateReward = 0.0;

    lastUsedNodePoolIndex = 1;
    return res;
}

SearchNode *THTS::createDecisionNode(double const &prob) {
    assert(lastUsedNodePoolIndex < nodePool.size());

    SearchNode *res = nodePool[lastUsedNodePoolIndex];

    if (res) {
        res->reset(prob, stepsToGoInNextState);
    } else {
        res = new SearchNode(prob, stepsToGoInNextState);
        nodePool[lastUsedNodePoolIndex] = res;
    }
    calcReward(states[stepsToGoInCurrentState], appliedActionIndex,
               res->immediateReward);

    ++lastUsedNodePoolIndex;


    return res;
}

SearchNode *THTS::createChanceNode(double const &prob, bool isActionNode) {
    assert(lastUsedNodePoolIndex < nodePool.size());

    SearchNode *res = nodePool[lastUsedNodePoolIndex];

    if (res) {
        res->reset(prob, stepsToGoInCurrentState);
    } else {
        res = new SearchNode(prob, stepsToGoInCurrentState);
        nodePool[lastUsedNodePoolIndex] = res;
    }

    ++lastUsedNodePoolIndex;

    res->isChanceNode = true;
    res->isActionNode = isActionNode;
    return res;
}

/******************************************************************
                       Parameter Setter
******************************************************************/

void THTS::setMaxSearchDepth(int _maxSearchDepth) {
    SearchEngine::setMaxSearchDepth(_maxSearchDepth);

    if (initializer) {
        initializer->setMaxSearchDepth(_maxSearchDepth);
    }
}

/******************************************************************
                            Print
******************************************************************/

void THTS::printStats(std::ostream &out, bool const &printRoundStats,
                      std::string indent) const {
    SearchEngine::printStats(out, printRoundStats, indent);

    if (currentTrial > 0) {
        out << indent << "Performed trials: " << currentTrial << std::endl;
        out << indent << "Created SearchNodes: " << lastUsedNodePoolIndex
            << std::endl;
        out << indent << "Cache Hits: " << cacheHits << std::endl;
        actionSelection->printStats(out, indent);
        outcomeSelection->printStats(out, indent);
        backupFunction->printStats(out, indent);
    }
    if (initializer) {
        initializer->printStats(out, printRoundStats, indent + "  ");
    }

    if (currentRootNode) {
        out << std::endl << indent << "Root Node: " << std::endl;
        currentRootNode->print(out);
        out << std::endl << "Q-Value Estimates: " << std::endl;
        for (unsigned int i = 0; i < currentRootNode->children.size(); ++i) {
            if (currentRootNode->children[i]) {
                out << indent;
                SearchEngine::actionStates[i].printCompact(out);
                out << ": ";
                currentRootNode->children[i]->print(out);
            }
        }
    }

    if (printRoundStats) {
        out << std::endl << indent << "ROUND FINISHED" << std::endl;
        out << indent << "Accumulated number of remaining steps in first "
                "solved root state: "
            << accumulatedNumberOfStepsToGoInFirstSolvedRootState << std::endl;
        out << indent << "Accumulated number of trials in root state: "
            << accumulatedNumberOfTrialsInRootState << std::endl;
        out << indent << "Accumulated number of search nodes in root state: "
            << accumulatedNumberOfSearchNodesInRootState << std::endl;
    }
}


/******************************************************************
                    generate Equivalence Class
******************************************************************/

void THTS::generateEquivalenceClass() {
    // std::cout <<"size is "<<pq.size() << std::endl;
    // std::cout <<"size of qvaluemean"<<SearchNode::qvalueMean.size() << std::endl;


    currentLevel = -1;
    currentLeaveLevel = -1;

    leaveEQCLass = 0;
    numberOfEQclasses = 0;
    leaveisChanceNode = true;
    currentIsChanceNode = true;

    qvalueSum.clear();
    qvalueNumbersOfEQClasses.clear();


    vectorChildrenOnLevel.clear();
    currentChildrenMap.clear();
    specialChildren.clear();
    tempMap.clear();

    // int test=0;
    for (SearchNode *const &currentNode : pq) {
        // std::cout << "current node steps to go are   "<<currentNode->stepsToGo<<" and is a ChanceNode "<<currentNode->isChanceNode<<std::endl;

        // if(currentLeaveLevel==-1){
        //    std::cout << "das niedrigste level ist  "<<currentNode->stepsToGo<<std::endl;
        // }
        //nodes that are leaves :
        if (currentNode->isALeafNode()) {
            //special case , where desicion node have no children and the reward is known
            if (!currentNode->isChanceNode && currentNode->children.size() == 0) {
                currentNode->equivalenceClassPos = numberOfEQclasses;
                qvalueNumbersOfEQClasses.push_back(1.0);
                qvalueSum.push_back(currentNode->immediateReward + currentNode->futureReward);
                numberOfEQclasses++;
            }
                //if it is a leaf node check on which level it is and save this level
            else if (currentNode->stepsToGo != currentLeaveLevel || leaveisChanceNode != currentNode->isChanceNode) {
                leaveisChanceNode = currentNode->isChanceNode;
                currentNode->equivalenceClassPos = numberOfEQclasses;
                leaveEQCLass = numberOfEQclasses;   //save the EQ class
                currentLeaveLevel = currentNode->stepsToGo;         //save the level so that other leaves on this level have the same number

                //   std::cout <<"------------new level leaf with leaveEQCLass  " <<leaveEQCLass<<"and step"<<currentLeaveLevel<< std::endl;

                qvalueNumbersOfEQClasses.push_back(1.0);
                qvalueSum.push_back(currentNode->immediateReward + currentNode->futureReward);

                numberOfEQclasses++;
            } else {
                currentNode->equivalenceClassPos = leaveEQCLass;
                assert(numberOfEQclasses > 0);
                assert(qvalueSum.size() == numberOfEQclasses);

                qvalueSum[numberOfEQclasses - 1] += currentNode->immediateReward + currentNode->futureReward;
                qvalueNumbersOfEQClasses[numberOfEQclasses - 1] += 1.0;

/*
                if(leaveEQCLass==0) {
                    std::cout << "------------ bottom leaf on level" << currentNode->stepsToGo <<" leaveEQCLass"<<leaveEQCLass<<" and is a chancenode "<<currentNode->isChanceNode <<std::endl;
                }else{
                    std::cout << "------------ old leaf on level" << currentNode->stepsToGo <<" leaveEQCLass"<<leaveEQCLass<<" and is a chancenode "<<currentNode->isChanceNode<< std::endl;
                }
*/
            }
        }
            //new level , declare new EQ class save the children map into the vector
            //the EQ class cannot be generated over multiple level if overlappingEQclass is true
        else if (currentLevel != currentNode->stepsToGo || currentIsChanceNode != currentNode->isChanceNode) {

            currentIsChanceNode = currentNode->isChanceNode;
            vectorChildrenOnLevel.clear();

            //create mapping of the child with level-prob

            currentNode->equivalenceClassPos = numberOfEQclasses;
            currentLevel = currentNode->stepsToGo;    //new level
            //std::cout <<"------------make new childrenmap vector , number of EQs is "<<numberOfEQclasses<<"// node stepstogo"<<currentNode->stepsToGo<< std::endl;

            vectorChildrenOnLevel.push_back(
                    makeChildrenOnLevel(currentNode));  // make children can different between chance and decision node

            qvalueNumbersOfEQClasses.push_back(1.0);
            qvalueSum.push_back(currentNode->immediateReward + currentNode->futureReward);

            numberOfEQclasses++;

        } else {
            //  std::cout <<"------------make children old level"<<currentLevel<<"// node stepstogo"<<currentNode->stepsToGo<< std::endl;
            currentChildrenMap.clear();
            currentChildrenMap = makeChildrenOnLevel(currentNode);
            // check the children maps of the other nodes on the same level , if there is a match
            //std::cout <<"compare vector  " <<currentNode->stepsToGo<< std::endl;
            isSameEQClass = false;
            for (auto &c:vectorChildrenOnLevel) {
                if (c.size() == currentChildrenMap.size()) {
                    isSameEQClass = true;

                    if (currentNode->isChanceNode) {  // if it is a chanceNode we don't want the children but the next decisionNodes
                        specialChildren.clear();
                        currentNode->collectAllDecisionNodeSuccessor(specialChildren);
                        for (unsigned int i = 0; i < specialChildren.size(); ++i) {
                            if (specialChildren[i].first) {
                                childEQ = specialChildren[i].first->equivalenceClassPos;
                                //if there is no same EQ or not the same value in EQ
                                if (!c.count(childEQ) || c.at(childEQ) != currentChildrenMap.at(childEQ)) {
                                    isSameEQClass = false;
                                    //     std::cout <<"not same "  <<std::endl;
                                    break;
                                }
                            }
                        }
                    } else {  // for decision nodes it is only necessary to check the children
                        for (unsigned int i = 0; i < currentNode->children.size(); ++i) {
                            if (currentNode->children[i]) {
                                childEQ = currentNode->children[i]->equivalenceClassPos;
                                //if there is no same EQ or not the same value in EQ
                                if (!c.count(childEQ) || c.at(childEQ) != currentChildrenMap.at(childEQ)) {
                                    isSameEQClass = false;
                                    //     std::cout <<"not same "  <<std::endl;
                                    break;
                                }
                            }
                        }
                    }

                    if (isSameEQClass) {
                        currentNode->equivalenceClassPos = c.at(-2);
                        // std::cout <<"same EQclass with the eqpos  " <<  currentNode->equivalenceClassPos<<std::endl;
                        qvalueSum[numberOfEQclasses - 1] += currentNode->immediateReward + currentNode->futureReward;
                        qvalueNumbersOfEQClasses[numberOfEQclasses - 1] += 1.0;

                        //      std::cout <<"gleichheit mit EQ1 " <<  currentNode->equivalenceClassPos<<std::endl;
                        break;
                    }
                }
            }
            //  std::cout <<" finished compare vector  " <<currentNode->stepsToGo<< std::endl;
            //no same children EQ
            if (!isSameEQClass) {
                currentNode->equivalenceClassPos = numberOfEQclasses;

                qvalueNumbersOfEQClasses.push_back(1.0);
                qvalueSum.push_back(currentNode->immediateReward + currentNode->futureReward);


                numberOfEQclasses++;
            } else {
                // std::cout <<"gleichheit mit EQ2 " <<  currentNode->equivalenceClassPos<<std::endl;
            }
            currentChildrenMap.at(-2) = currentNode->equivalenceClassPos;
            vectorChildrenOnLevel.push_back(currentChildrenMap);
        }

        //Debugging if this is true , this   Node is uninitialized
        if (currentNode->equivalenceClassPos == -1) {
            std::cout << "#################FAIL##############" << std::endl;
            std::cout << "FAIL" << currentNode->equivalenceClassPos << std::endl;
            std::cout << "FAIL is ChanceNode " << currentNode->isChanceNode << std::endl;
            std::cout << "FAIL is same EQClass " << isSameEQClass << std::endl;
            std::cout << "FAIL vector size  " << vectorChildrenOnLevel.size() << std::endl;
            std::cout << "#################FAIL##############" << std::endl;
            assert(false);
        }

    }
    makeQmean();    //here the vector is generated for the Qmean with vector qsum and qnumberofEqclass
    //std::cout <<"finished generating there are " <<numberOfEQclasses <<"classes "<<std::endl;
}


std::map<int, double> THTS::makeChildrenOnLevel(SearchNode *node) {
    //std::cout <<"entering makeChildrenOnLevel on level "<<node->stepsToGo <<" and with ischancenode "<<node->isChanceNode<<std::endl;
    tempMap.clear();
    if (!node->isChanceNode) {
        //create the children EQclass-probability map for the currentNode
        //  std::cout <<"making oof children with size " <<currentNode->children.size()<< std::endl;
        for (SearchNode *child : (node->children)) {
            if (child) {
                //    std::cout <<"child with  " <<  child->equivalenceClassPos<<std::endl;
                if (tempMap.count(child->equivalenceClassPos)) {
                    //already a child with the same EQclass
                    //   std::cout << "same with EQ class: " <<child->equivalenceClassPos <<" //"<<currentNode->children.size()<<" and is chancenode"<<child->isChanceNode<<std::endl;
                    tempMap.at(child->equivalenceClassPos) += child->prob;

                } else {
                    //new EQclass
                    //            std::cout << "new with EQclass" <<child->equivalenceClassPos <<" size of children "<<node->children.size()<<
                    //                        " chidl steps "<<child->stepsToGo<< " and isChancenode "<<child->isChanceNode
//                          <<" and is  a leaf: "<<child->isALeafNode()<<std::endl;

                    tempMap.insert(std::make_pair(child->equivalenceClassPos, child->prob));

                }
                assert(child->equivalenceClassPos != -1);
                //if(child->equivalenceClassPos==-1){
                //std::cout << "#################################child is ChanceNode " <<child->isChanceNode<<" and EQ " <<child->equivalenceClassPos <<" and level "<<child->stepsToGo<<std::endl;
                //  std::cout << "#################################parent isChanceNode " <<node->isChanceNode <<" and EQ " <<node->equivalenceClassPos<<" and level "<<node->stepsToGo<<std::endl;

                //}
            } else {
                //std::cout <<"child not exist  " <<std::endl;
            }
        }
    } else {
        // is a ChanceNode , here not the children are scanned but the level with the decisionnode(so the rekursiv chancenodes children)
        specialChildren.clear();
        assert(specialChildren.size() == 0);
        node->collectAllDecisionNodeSuccessor(specialChildren);
        for (unsigned int i = 0; i < specialChildren.size(); ++i) {
            if (specialChildren[i].first) {
                if (tempMap.count(specialChildren[i].first->equivalenceClassPos)) {
                    //already a child with the same EQclass
                    tempMap.at(
                            specialChildren[i].first->equivalenceClassPos) += specialChildren[i].second;//add the probability

                } else {
                    //new EQclass
                    //  std::cout << "new with EQclass " <<specialChildren[i].first->equivalenceClassPos <<" size of specialchildren "<<specialChildren.size()<<
                    //          " child steps "<<specialChildren[i].first->stepsToGo<< " and isChancenode "<<specialChildren[i].first->isChanceNode
                    //        <<" and is  a leaf: "<<specialChildren[i].first->isALeafNode()<<std::endl;

                    tempMap.insert(
                            std::make_pair(specialChildren[i].first->equivalenceClassPos, specialChildren[i].second));

                }

                //ERROR
                  if(specialChildren[i].first->equivalenceClassPos==-1){
                    std::cout << "#################################child is ChanceNode " <<specialChildren[i].first->isChanceNode<<" and EQ " <<specialChildren[i].first->equivalenceClassPos <<" and level "<<specialChildren[i].first->stepsToGo<<std::endl;
                      std::cout << "#################################child is a leaf " <<specialChildren[i].first->isALeafNode()<<" and has prob  " <<specialChildren[i].second <<std::endl;
                      std::cout << "#################################parent isChanceNode " <<node->isChanceNode <<" and EQ " <<node->equivalenceClassPos<<" and level "<<node->stepsToGo<<std::endl;
                      std::cout << "#################################parent  is a leaf " <<node->isALeafNode() <<std::endl;

                      std::cout << "#################################current level is  "<<currentLevel<<" and current leaf level is "<<currentLeaveLevel <<std::endl;

                      std::cout << "#################################cspecial children size is  "<<specialChildren.size()<<" and current i is : "<<i <<std::endl;
                      std::cout << "#################################normal children size is    "<<node->children.size()<< std::endl;


                  }
                assert(specialChildren[i].first->equivalenceClassPos != -1);
            } else {
                //std::cout <<"child not exist  " <<std::endl;
            }
        }
    }
    //std::cout<< "  finished adding" << std::endl;

    //add the information of the parent , note this is -1 if not initialize
    tempMap.insert(std::make_pair(-2, node->equivalenceClassPos));
    return tempMap;
}

//generate the QValue of the EQ classes
void THTS::makeQmean() {
    assert(qvalueSum.size() > 0);
    assert(qvalueNumbersOfEQClasses.size() == qvalueSum.size());
    SearchNode::qvalueMean.clear();

    for (unsigned int i = 0; i < qvalueSum.size(); ++i) {
        //std::cout <<"current level is "<<i<<" the sum here is: " <<qvalueSum[i]<<std::endl;
        //std::cout <<"and it's size (number of classes in it)  " <<qvalueNumbersOfEQClasses[i]<<std::endl;
        SearchNode::qvalueMean.push_back(qvalueSum[i] / qvalueNumbersOfEQClasses[i]);
    }

    //std::cout <<"all the Q-value means and size is "<<SearchNode::qvalueMean.size() <<std::endl;
    /* for(auto &v:SearchNode::qvalueMean){
         std::cout <<" " <<v<<std::endl;
     }*/
    // std::cout <<"finished qvalue-mean " <<std::endl;
}
