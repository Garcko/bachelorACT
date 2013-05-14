#include "ippc_client.h"

#include "prost_planner.h"

#include "utils/system_utils.h"
#include "utils/string_utils.h"
#include "utils/strxml.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

using namespace std;

bool IPPCClient::init() {
    assert(socket == -1);
    try {
        socket = connectToServer();
        if(socket <= 0) {
            return false;
        }
    } catch(const exception& e) {
        cerr << endl << "IPPCClient: " << e.what() << endl;
        return false;
    } catch(...) {
        cerr << "IPPCClient: fatal error" << endl;
        return false;
    }
    return true;
}

bool IPPCClient::run(string& dir, string& problemName, string& plannerDesc) {
    //read domain and problem files
    string domain;
    string problem;
    if(!readFiles(dir,problemName,domain,problem)) {
        return false;
    }

    //request round
    int remainingTime = -1;
    if(!initSession(problemName,remainingTime)) {
        return false;
    }

    //init planner
    ProstPlanner* planner = ProstPlanner::fromString(plannerDesc, domain, problem, numberOfRounds);
    planner->init(stateVariableIndices);
    vector<double> nextState(stateVariableIndices.size());

    //main loop
    for(int i = 0; i < numberOfRounds; ++i) {
        if(!initRound(nextState)) {
            return false;
        }
        planner->initNextRound();

        while(true) {
            vector<string> nextActions = planner->plan(nextState);
            if(!submitAction(nextActions,nextState)) {            
                break;
            }
        }
    }

    return true;
}

bool IPPCClient::readFiles(string& dir, string& problemName, string& domain, string& problem) {
    int index = (int)problemName.find(("_inst"));
    string domainFileName = dir + problemName.substr(0,index) + "_mdp.rddl_prefix";
    string problemFileName = dir + problemName + ".rddl_prefix";

    cout << "opening files " << domainFileName << " / " << problemFileName << endl;

    if(!SystemUtils::readFile(domainFileName, domain, "//")) {
        cout << "cannot find file " << domainFileName << endl;
        return false;
    }
    if(!SystemUtils::readFile(problemFileName, problem, "//")) {
        cout << "cannot find file " << problemFileName << endl;
        return false;
    }
    return true;
}

bool IPPCClient::stop() {
    cout << "***********************************************" << endl;
    cout << ">>>            END OF SESSION                  " << endl;
    cout << ">>>           TOTAL REWARD: " << accumulatedReward << endl;
    cout << ">>>          AVERAGE REWARD: " << (double)(accumulatedReward / (double) numberOfRounds) << endl;
    cout << "***********************************************\n" << endl;

    if(socket == -1) {
        return false;
    }
    close(socket);
    return true;
}

/******************************************************************************
                               Server Communication
******************************************************************************/

int IPPCClient::connectToServer() {
    struct hostent* host = ::gethostbyname(hostName.c_str());
    if(!host) {
        return -1;
    }

    int res = ::socket(PF_INET, SOCK_STREAM, 0);
    if(res == -1) {
        return -1;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr = *((struct in_addr*) host->h_addr);
    memset(&(addr.sin_zero), '\0', 8);

    if(::connect(res, (struct sockaddr*) &addr, sizeof(addr)) == -1) {
        return -1;
    }
    return res;
}

bool IPPCClient::initSession(string& rddlProblem, int& remainingTime) {
    stringstream os;
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << "<session-request>"
            << "<problem-name>" << rddlProblem << "</problem-name>"
            << "<client-name>" << "prost" << "</client-name>"
            << "<no-header/>" << "</session-request>" << '\0';
    if(write(socket, os.str().c_str(), os.str().length()) == -1) {
        return false;
    }

    const XMLNode* serverResponse = XMLNode::readNode(socket);
    if(!serverResponse) {
        return false;
    }

    string s;
    if(!serverResponse->dissect("num-rounds", s)) {
        delete serverResponse;
        return false;
    }
    numberOfRounds = atoi(s.c_str());

    if(!serverResponse->dissect("time-allowed", s)) {
        delete serverResponse;
        return false;
    }
    remainingTime = atoi(s.c_str());

    delete serverResponse;
    return true;
}

bool IPPCClient::initRound(vector<double>& initialState) {
    stringstream os;
    os.str("");
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<round-request/>" << '\0';

    if(write(socket, os.str().c_str(), os.str().length()) == -1) {
        return false;
    }

    const XMLNode* serverResponse = XMLNode::readNode(socket);

    if(!serverResponse || serverResponse->getName() != "round-init") {
        cerr << "Error in server's round-request response" << endl;
        delete serverResponse;
        return false;
    }
    delete serverResponse;

    serverResponse = XMLNode::readNode(socket);
    readState(serverResponse, initialState);

    delete serverResponse;
    return true;
}

bool IPPCClient::submitAction(vector<string>& actions, vector<double>& nextState) {
    stringstream os;
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" << "<actions>";

    for(unsigned int i = 0; i < actions.size(); ++i) {
        size_t cutPos = actions[i].find("(");
        assert(cutPos != string::npos);
        string actionName = actions[i].substr(0,cutPos);
        os << "<action><action-name>" << actionName << "</action-name>";
        string allParams = actions[i].substr(cutPos+1);
        assert(allParams[allParams.length()-1] == ')');
        allParams = allParams.substr(0,allParams.length()-1);
        vector<string> params;
        StringUtils::split(allParams, params, ",");
        for(unsigned int j = 0; j < params.size(); ++j) {
            StringUtils::trim(params[j]);
            os << "<action-arg>" << params[j] << "</action-arg>";
        }
        os << "<action-value>true</action-value></action>";
    }
    os << "</actions>" << '\0';
    if(write(socket, os.str().c_str(), os.str().length()) == -1) {
        return false;
    }

    const XMLNode* serverResponse = XMLNode::readNode(socket);

    if(serverResponse->getName() == "round-end" || serverResponse->getName() == "end-session") {
        string s;
        if(serverResponse->dissect("round-reward", s)) {
            double reward = atof(s.c_str());
            cout << "***********************************************" << endl;
            cout << ">>> END OF ROUND -- REWARD RECEIVED: " << reward << endl;
            cout << "***********************************************\n" << endl;
            accumulatedReward += reward;
        }
        delete serverResponse;
        return false;
    }

    readState(serverResponse, nextState);

    delete serverResponse;
    return true;
}

void IPPCClient::readState(const XMLNode* node, vector<double>& nextState) {
    assert(node);
    assert(node->getName() == "turn");

    if(node->size() == 2 && node->getChild(1)->getName() == "no-observed-fluents") {
        assert(false);
    }

    map<string, double> newValues;

    for(int i = 0; i < node->size(); i++) {
        const XMLNode* child = node->getChild(i);
        if(child->getName() == "observed-fluent") {
            readVariable(child, newValues);
        }
    }

    for(map<string,double>::iterator it = newValues.begin(); it != newValues.end(); ++it) {
        if(stateVariableIndices.find(it->first) != stateVariableIndices.end()) {
            nextState[stateVariableIndices[it->first]] = it->second;
        }
    }
}

void IPPCClient::readVariable(const XMLNode* node, map<string, double>& result) {
    string name;
    if(node->getName() != "observed-fluent") {
        assert(false);
    }

    if(!node->dissect("fluent-name", name)) {
        assert(false);
    }
    name = name.substr(0,name.length()-1);

    vector<string> params;
    double value = 0.0;

    for(int i = 0; i < node->size(); i++) {
        const XMLNode* paramNode = node->getChild(i);
        if(!paramNode) {
            assert(false);
            continue;
        } else if(paramNode->getName() == "fluent-arg") {
            string param = paramNode->getText();
            params.push_back(param.substr(0,param.length()-1));
        } else if(paramNode->getName() == "fluent-value") {
            string res = paramNode->getText();
            res = res.substr(0,res.length()-1);
            if(res =="true") {
                value = 1.0;
            } else if(res == "false") {
                value = 0.0;
            } else {
                value = atof(res.c_str());
            }
        }
    }
    name += "(";
    for(unsigned int i = 0; i < params.size(); ++i) {
        name += params[i];
        if(i != params.size()-1) {
            name += ", ";
        }
    }
    name += ")";
    assert(result.find(name) == result.end());
    result[name] = value;
}
