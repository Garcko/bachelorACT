#! /usr/bin/env python
# -*- coding: latin-1 -*-

# This script creates an overview of results of different planner
# configurations; IPPC scores are based only on all available planners (i.e.,
# there is no min-algorithm). Careful, it is possible that all configuration
# perform similar or that small result differences lead to huge IPPC score
# differences.

import os
import sys
import latex
import summarize_results

import xml.etree.ElementTree as ET

printFatThreshold = 0.95
totalScorePrintFatThreshold = 0.9

class Domain:
    def __init__(self, name):
        self.name = name
        self.instances = list()

    def addInstance(self, inst):
        self.instances.append(inst)

    def finalizeInitialization(self):
        self.instances.sort(key=lambda Instance : Instance.name)
        instDict = dict()
        for inst in self.instances:
            instDict[inst.name] = inst
        self.instances = instDict

    def calcValues(self):
        for inst in self.instances:
            self.instances[inst].calcValue()

    def printDomain(self):
        for inst in self.instances:
            self.instances[inst].printInstance()

class Instance:
    def __init__(self, domain, name, minValue):
        self.domain = domain
        self.name = name
        self.minValue = float('inf')
        self.maxValue = float('-inf')
        self.normFactor = None
        self.results = dict()
        self.confidence95 = dict()
        self.times = dict()

    def calcValue(self):
        for planner in self.results:
            if self.results[planner] > self.maxValue:
                self.maxValue = self.results[planner]
            if self.results[planner] < self.minValue:
                self.minValue = self.results[planner]

        self.normFactor = self.maxValue - self.minValue 

    def getIPPCScore(self, plannerName):
        assert plannerName in self.results

        if self.results[plannerName] < self.minValue:
            return 0.0
        elif self.normFactor == 0:
            return 1.0

        return round(float((self.results[plannerName] - self.minValue) / self.normFactor),2)

    def printInstance(self):
        print self.domain.name + " " + str(self.name) + " (Min: " + str(self.minValue) + ", Max: " + str(self.maxValue) + ")"
        for planner in self.results:
            print planner + ": " +  str(self.results[planner])

class Planner:
    def __init__(self, name, totalTime):
        self.name = name
        self.totalTime = totalTime
        self.timesPerDomain = dict()
        self.rewards = dict()
        self.confidence95 = dict()
        self.times = dict()

def initDomains(minDirName):
    assert os.path.isdir(minDirName)

    files = os.listdir(minDirName)
    assert "result.xml" in files

    domains = list()
    
    tree = ET.parse(minDirName + "/result.xml")
    root = tree.getroot()
    for domChild in root.findall('Domain'):
        domainName = domChild.find('DomainName').text
        domain = Domain(domainName)
        domains.append(domain)

        for probChild in domChild.findall('Problem'):
            problemName = int(probChild.find('ProblemName').text)
            minValue = float(probChild.find('AvgReward').text)
            instance = Instance(domain, problemName, minValue)
            domain.addInstance(instance)

    for dom in domains:
        dom.finalizeInitialization()

    domains.sort()
    domRes = dict()
    for dom in domains:
        domRes[dom.name] = dom

    return domRes

def readResults(dirName, domains, planners):
    if not os.path.isdir(dirName):
        return domains, planners

    files = os.listdir(dirName)
    if not "result.xml" in files:
        return domains, planners

    tree = ET.parse(dirName + "/result.xml")
    root = tree.getroot()

    name = root.find('PlannerName').text
    totalTime = float(root.find('Time').text)

    planner = Planner(name, totalTime)
    planners.append(planner)

    for domChild in root.findall('Domain'):
        domainName = domChild.find('DomainName').text
        assert domainName in domains
        planner.timesPerDomain[domainName] = float(domChild.find('Time').text)
        planner.rewards[domainName] = dict()
        planner.confidence95[domainName] = dict()
        planner.times[domainName] = dict()
        domain = domains[domainName]

        for probChild in domChild.findall('Problem'):
            problemName = int(probChild.find('ProblemName').text)
            assert problemName in domain.instances
            instance = domain.instances[problemName]

            reward = float(probChild.find('AvgReward').text)
            instance.results[planner.name] = reward
            planner.rewards[domainName][problemName] = reward

            conf95 = float(probChild.find('Confidence95').text)
            instance.confidence95[planner.name] = conf95
            planner.confidence95[domainName][problemName] = conf95

            time = float(probChild.find('Time').text)
            instance.times[planner.name] = time
            planner.times[domainName][problemName] = time

    planners.sort(key=lambda Planner: Planner.name)

    return domains, planners

def printResults(domains, planners, outFile):
    doc = latex.Document("article")
    doc.add_package("geometry", "a3paper", "margin=2.5cm", "landscape")
    doc.add_package("scrtime")
    doc.add_package("amsmath")
    doc.add_package("amssymb")
    doc.add_package("color")
    doc.add_package("colortbl")

    doc.add_definition(r"\setlength{\parindent}{0cm}")
    doc.add(r"\section*{PROST: \today\ \thistime}")

    for domainName in domains:
        doc.add(r"\subsection*{Rewards: %s}" % domainName)
        addRewardTable(doc, domains[domainName], planners)

        doc.add(r"\subsection*{IPPC Scores: %s}" % domainName)
        addIPPCScoreTable(doc, domains[domainName], planners)

    doc.add(r"\subsection*{Total Time (in minutes)}")
    addTotalTimeTable(doc, domains, planners)

    doc.add(r"\subsection*{IPPC Scores: Total}")
    addTotalIPPCScoreTable(doc, domains, planners)

    f = open(outFile, 'w+')
    doc.render(f)
    f.close()

def addTotalTimeTable(doc, domains, planners):
    table_style = (["|", "l", "|"] +
                   ["r", r"@{$\pm$}", "r"] * (len(domains)+1) +
                   ["|"])
    table = latex.Table(table_style)
    head = table.head()
    head.add_hline()
    head.add_cell(r"")

    for domainName in domains:
        head.add_cell(domainName, span=2, align="c")
    head.add_cell("Total", span=2, align="c")
    head.add_row()
    head.add_hline()
    head.add_hline()
    table.foot().add_hline()

    for planner in planners:
        table.add_cell(planner.name)

        numberOfInstances = 0
        for domainName in domains:
            if planner.timesPerDomain[domainName] < 0.0:
                table.add_cell("n/a", span=2, align="c")
            else:
                table.add_cell(str(round(planner.timesPerDomain[domainName]/60.0,2)), span=2, align="c")
        if planner.totalTime < 0.0:
            table.add_cell("n/a", span=2, align="c")
        else:
            table.add_cell(str(round(planner.totalTime/60.0,2)), span=2, align="c")
        table.add_row()

    doc.add(table, r"\bigskip") 

def addTotalIPPCScoreTable(doc, domains, planners):
    table_style = (["|", "l", "|"] +
                   ["r", r"@{$\pm$}", "r"] * (len(domains)+1) +
                   ["|"])
    table = latex.Table(table_style)
    head = table.head()
    head.add_hline()
    head.add_cell(r"")

    for domainName in domains:
        head.add_cell(domainName, span=2, align="c")
    head.add_cell("Total", span=2, align="c")
    head.add_row()
    head.add_hline()
    head.add_hline()
    table.foot().add_hline()

    for planner in planners:
        table.add_cell(planner.name)

        IPPCSum = float(0.0)
        numberOfInstances = 0
        for domainName in domains:
            domainSum = float(0.0)
            for instanceName in domains[domainName].instances:
                numberOfInstances += 1
                domainSum += domains[domainName].instances[instanceName].getIPPCScore(planner.name)
                IPPCSum += domains[domainName].instances[instanceName].getIPPCScore(planner.name)
            domainSum = round(float(domainSum/float(len(domains[domainName].instances))),2)
            if domainSum == 1.0:
                table.add_cell("\\textbf{\\textcolor{red}{"+str(domainSum)+"}}", span=2, align="c")
            elif domainSum > printFatThreshold:
                table.add_cell("\\textbf{"+str(domainSum)+"}", span=2, align="c")
            else:
                table.add_cell(str(domainSum), span=2, align="c")

        IPPCSum = round(float(IPPCSum / (float(numberOfInstances))),2)
        if IPPCSum >= 0.98:
            table.add_cell("\\textbf{\\textcolor{red}{"+str(IPPCSum)+"}}", span=2, align="c")
        elif IPPCSum > totalScorePrintFatThreshold:
            table.add_cell("\\textbf{"+str(IPPCSum)+"}", span=2, align="c")
        else:
            table.add_cell(str(IPPCSum), span=2, align="c")
        table.add_row()

    doc.add(table, r"\bigskip")        

def addIPPCScoreTable(doc, domain, planners):
    table_style = (["|", "l", "|"] +
                   ["r", r"@{$\pm$}", "r"] * len(domain.instances) +
                   ["|"])
    table = latex.Table(table_style)
    head = table.head()
    head.add_hline()
    head.add_cell(r"")

    for instanceName in domain.instances:
        head.add_cell(instanceName, span=2, align="c")
    head.add_row()
    head.add_hline()
    head.add_hline()
    table.foot().add_hline()

    for planner in planners:
        table.add_cell(planner.name)
        for instanceName in domain.instances:
            score = domain.instances[instanceName].getIPPCScore(planner.name)
            if score == 1.0:
                table.add_cell("\\textbf{\\textcolor{red}{"+str(score)+"}}", span=2, align="c")
            elif score > printFatThreshold:
                table.add_cell("\\textbf{"+str(score)+"}", span=2, align="c")
            else:
                table.add_cell(str(score), span=2, align="c")
        table.add_row()

    doc.add(table, r"\bigskip")

def addRewardTable(doc, domain, planners):
    table_style = (["|", "l", "|"] +
                   ["r", r"@{$\pm$}", "r"] * len(domain.instances) +
                   ["|"])
    table = latex.Table(table_style)
    head = table.head()
    head.add_hline()
    head.add_cell(r"")

    for instanceName in domain.instances:
        head.add_cell(instanceName, span=2, align="c")
    head.add_row()
    head.add_hline()
    head.add_hline()
    table.foot().add_hline()

    for planner in planners:
        table.add_cell(planner.name)
        for instanceName in domain.instances:
            reward = domain.instances[instanceName].results[planner.name]
            score = domain.instances[instanceName].getIPPCScore(planner.name)
            conf95 = domain.instances[instanceName].confidence95[planner.name]
            if score == 1.0:
                table.add_cell("\\textbf{\\textcolor{red}{"+str(reward)+"($\pm$"+str(conf95)+")}}", span=2, align="c")
            elif score > printFatThreshold:
                table.add_cell("\\textbf{$"+str(reward)+"(\\pm"+str(conf95)+")$}", span=2, align="c")
            else:
                table.add_cell("$"+str(reward)+"(\\pm"+str(conf95)+")$", span=2, align="c")
        table.add_row()

    doc.add(table, r"\bigskip")

def analyzeResults(directory, outFile, compress):
    planners = list()
    domains = initDomains(directory+"/min")

    #summarize_results.summarizeResults(directory, compress)
    resultDirNames = os.listdir(directory)
    for resultDirName in resultDirNames:
        print resultDirName
        domains, plannerNames = readResults(directory+resultDirName, domains, planners)

    for d in domains:
        domains[d].calcValues()

    printResults(domains, planners, outFile)

if __name__ == "__main__":
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print >> sys.stderr, "Usage: analyze_results.py <resultsDir> <outFile> [--tar]"
        exit()

    if len(sys.argv) == 4 and sys.argv[3] == "--tar":
        analyzeResults(sys.argv[1], sys.argv[2], True)
    else:
        analyzeResults(sys.argv[1], sys.argv[2], False)

          
