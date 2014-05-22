#include <gtest/gtest.h>
#include <map>
#include <vector>
#include <fcntl.h>
#include "../../search/ippc_client.h"
#include "../../search/utils/strxml.h"

using std::string;
using std::map;
using std::vector;

const XMLNode* createXMLNodeFromString(const char* input) {
    FILE* xmlContent = fopen("xml", "w");
    fprintf(xmlContent, "%s", input);
    fclose(xmlContent);
    int fd = open("xml", O_RDONLY);
    const XMLNode* node = XMLNode::readNode(fd);
    remove("xml");
    return node;
}

TEST(ippcClientTest, testReadState) {
}

TEST(ippcClientTest, testReadVariable) {
    std::map<string, int> stateVariableIndices;
    vector<vector<string> > stateVariableValues;
    IPPCClient* client = new IPPCClient(NULL, "host", 10,
            stateVariableIndices, stateVariableValues);
    const XMLNode* corruptedNode =
        createXMLNodeFromString("<nonFluent>content</nonFluent>");
    std::map<std::string, std::string> result;
}
