#ifndef LOGICAL_EXPRESSIONS_H
#define LOGICAL_EXPRESSIONS_H

#include "typed_objects.h"
#include "state.h"
#include "actions.h"

#include <set>

class RDDLParser;
class Instantiator;

class LogicalExpression {
public:
    virtual ~LogicalExpression() {}

    virtual LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    virtual LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    virtual LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    virtual LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    virtual void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                                    std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    virtual void calculateDomain(ActionState const& actions, std::set<double>& res);

    virtual void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    virtual void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    virtual void print() = 0;
};



/*****************************************************************
                       Schematic Stuff
*****************************************************************/

class VariableDefinition {
public:
    static void parse(std::string& desc, UnprocessedPlanningTask* task);

    static VariableDefinition* rewardInstance() {
        static VariableDefinition* rewardInst = new VariableDefinition("reward", std::vector<ObjectType*>(), VariableDefinition::STATE_FLUENT, RealType::instance());
        return rewardInst;
    }

    std::string name;
    std::vector<ObjectType*> params;

    enum VariableType {
        STATE_FLUENT, 
        ACTION_FLUENT, 
        INTERM_FLUENT, 
        NON_FLUENT} variableType;

    Type* valueType;
    double defaultValue;
    int level;

    void print();

private:
    VariableDefinition(std::string _name, std::vector<ObjectType*> _params, VariableType _variableType, Type* _valueType, double _defaultValue = 0.0, int _level = 1) :
        name(_name), params(_params), variableType(_variableType), valueType(_valueType), defaultValue(_defaultValue), level(_level) {}
};

class StateActionConstraint {
public:
    static void parse(std::string& desc, UnprocessedPlanningTask* task, RDDLParser* parser);

    LogicalExpression* sac;

    void replaceQuantifier(UnprocessedPlanningTask* task, Instantiator* instantiator);
    void instantiate(UnprocessedPlanningTask* task);
    bool simplify(UnprocessedPlanningTask* task, std::map<StateFluent*, NumericConstant*>& replacements);

    void print();

private:
    StateActionConstraint(LogicalExpression* _sac) :
        sac(_sac) {}
};

class ParameterDefinition : public LogicalExpression {
public:
    std::string parameterName;
    ObjectType* parameterType;

    ParameterDefinition(std::string _parameterName, ObjectType* _parameterType):
        parameterName(_parameterName), parameterType(_parameterType) {}

    void print();
};

class ParameterDefinitionSet : public LogicalExpression {
public:
    std::vector<ParameterDefinition*> parameterDefs;

    ParameterDefinitionSet(std::vector<ParameterDefinition*>& _defs) :
        parameterDefs(_defs) {}
    ParameterDefinitionSet() {}

    void print();
};

/*****************************************************************
                      Variables & Constants
*****************************************************************/

class UninstantiatedVariable : public LogicalExpression {
public:
    UninstantiatedVariable(VariableDefinition* _parent, std::vector<std::string> _params);

    VariableDefinition* parent;

    std::string name;
    std::vector<std::string> params;

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);

    void print();
};

class AtomicLogicalExpression : public LogicalExpression {
public:
    static void parse(std::string& desc, UnprocessedPlanningTask* task);

    VariableDefinition* parent;
    std::string name;
    std::vector<Object*> params;

    double initialValue;
    int index;

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);
    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void print();

protected:
    AtomicLogicalExpression(VariableDefinition* _parent, std::vector<Object*> _params, double _initialValue);
};

class StateFluent : public AtomicLogicalExpression {
public:
    static StateFluent* rewardInstance() {
        static StateFluent* rewardInst = new StateFluent(VariableDefinition::rewardInstance(), std::vector<Object*>());
        return rewardInst;
    }

    StateFluent(VariableDefinition* _parent, std::vector<Object*> _params, double _initialValue) :
        AtomicLogicalExpression(_parent, _params, _initialValue) {}
    StateFluent(VariableDefinition* _parent, std::vector<Object*> _params) :
        AtomicLogicalExpression(_parent, _params, _parent->defaultValue) {}

    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);
    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);
};

class ActionFluent : public AtomicLogicalExpression {
public:
    ActionFluent(VariableDefinition* _parent, std::vector<Object*> _params) :
        AtomicLogicalExpression(_parent, _params, 0.0) {}

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);
};

class NonFluent : public AtomicLogicalExpression {
public:
    NonFluent(VariableDefinition* _parent, std::vector<Object*> _params, double _initialValue) :
        AtomicLogicalExpression(_parent, _params, _initialValue) {}
    NonFluent(VariableDefinition* _parent, std::vector<Object*> _params) :
        AtomicLogicalExpression(_parent, _params, _parent->defaultValue) {}
};

class NumericConstant : public LogicalExpression {
public:
    static NumericConstant* truth() {
        static NumericConstant* truthInst = new NumericConstant(1.0);
        return truthInst;
    }

    static NumericConstant* falsity() {
        static NumericConstant* falsityInst = new NumericConstant(0.0);
        return falsityInst;
    }

    double value;

    NumericConstant(double _value) :
        value(_value) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

/*****************************************************************
                    Quantifier & Subclasses
*****************************************************************/

class Quantifier : public LogicalExpression {
public:
    ParameterDefinitionSet* parameterDefsSet;
    LogicalExpression* expr;

    Quantifier(std::vector<LogicalExpression*>& _exprs);

    void getReplacements(UnprocessedPlanningTask* task, std::vector<std::string>& parameterNames, std::vector<std::vector<Object*> >& replacements, Instantiator* instantiator);

    void print();
};

class Sumation : public Quantifier {
public:
    Sumation(std::vector<LogicalExpression*>& _exprs) :
        Quantifier(_exprs) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);

    void print();
};

class Product : public Quantifier {
public:
    Product(std::vector<LogicalExpression*>& _exprs) :
        Quantifier(_exprs) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);

    void print();    
};

class UniversalQuantification : public Quantifier {
public:
    UniversalQuantification(std::vector<LogicalExpression*>& _exprs) :
        Quantifier(_exprs) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);

    void print();
};

class ExistentialQuantification: public Quantifier {
public:
    ExistentialQuantification(std::vector<LogicalExpression*>& _exprs) :
        Quantifier(_exprs) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);

    void print();
};

/*****************************************************************
                    Connective & Subclasses
*****************************************************************/

class Connective : public LogicalExpression {
public:
    std::vector<LogicalExpression*> exprs;
    double exprRes;

    Connective(std::vector<LogicalExpression*>& _exprs) :
        exprs(_exprs), exprRes(0.0) {}

    void print();
};

class Conjunction : public Connective {
public:
    Conjunction(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class Disjunction : public Connective {
public:
    Disjunction(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class EqualsExpression : public Connective {
public:
    EqualsExpression(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class GreaterExpression : public Connective {
public:
    GreaterExpression(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class LowerExpression : public Connective {
public:
    LowerExpression(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class GreaterEqualsExpression : public Connective {
public:
    GreaterEqualsExpression(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class LowerEqualsExpression : public Connective {
public:
    LowerEqualsExpression(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class Addition : public Connective {
public:
    Addition(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class Subtraction : public Connective {
public:
    Subtraction(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class Multiplication : public Connective {
public:
    Multiplication(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class Division : public Connective {
public:
    Division(std::vector<LogicalExpression*>& _exprs) :
        Connective(_exprs) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

/*****************************************************************
                   Probability Distributions
*****************************************************************/

class ProbabilityDistribution : public LogicalExpression {
public:
    LogicalExpression* expr;
    double randNum;

    ProbabilityDistribution(LogicalExpression* _expr) :
        expr(_expr), randNum(0.0) {}
};

class BernoulliDistribution : public ProbabilityDistribution {
public:
    BernoulliDistribution(LogicalExpression* _expr) :
        ProbabilityDistribution(_expr) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents, 
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class KronDeltaDistribution : public ProbabilityDistribution {
public:
    KronDeltaDistribution(LogicalExpression* _expr) :
        ProbabilityDistribution(_expr) {}

    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    void print();
};

/*****************************************************************
                     IfThenElse and Negate
*****************************************************************/

class IfThenElseExpression : public LogicalExpression {
public:
    LogicalExpression* condition;
    LogicalExpression* valueIfTrue;
    LogicalExpression* valueIfFalse;
    double exprRes;

    IfThenElseExpression(LogicalExpression* _condition, LogicalExpression* _valueIfTrue, LogicalExpression* _valueIfFalse) :
        condition(_condition), valueIfTrue(_valueIfTrue), valueIfFalse(_valueIfFalse), exprRes(0.0) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents,
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class MultiConditionChecker : public LogicalExpression {
public:
    std::vector<LogicalExpression*> conditions;
    std::vector<LogicalExpression*> effects;
    bool hasUncertainCondition;
    double exprRes;

    MultiConditionChecker(std::vector<LogicalExpression*> _conditions, std::vector<LogicalExpression*> _effects) :
        conditions(_conditions), effects(_effects), hasUncertainCondition(false), exprRes(0.0) {}

    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);
    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents,
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);

    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};

class NegateExpression : public LogicalExpression {
public:
    LogicalExpression* expr;

    NegateExpression(LogicalExpression* _expr) :
        expr(_expr) {}

    LogicalExpression* replaceQuantifier(UnprocessedPlanningTask* task, std::map<std::string, std::string>& replacements, Instantiator* instantiator);
    LogicalExpression* instantiate(UnprocessedPlanningTask* task, std::map<std::string, Object*>& replacements);
    LogicalExpression* simplify(UnprocessedPlanningTask* task, std::map<StateFluent*,NumericConstant*>& replacements);

    void collectInitialInfo(bool& isProbabilistic, std::vector<StateFluent*>& dependentStateFluents,
                            std::vector<ActionFluent*>& positiveDependentActionFluents, std::vector<ActionFluent*>& negativeDependentActionFluents);
    LogicalExpression* determinizeMostLikely(NumericConstant* randomNumberReplacement);

    void calculateDomain(ActionState const& actions, std::set<double>& res);

    void evaluate(double& res, State const& current, State const& next, ActionState const& actions);
    void evaluateToKleeneOutcome(double& res, State const& current, State const& next, ActionState const& actions);

    void print();
};



#endif
