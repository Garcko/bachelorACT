LogicalExpression* LogicalExpression::determinizeMostLikely(NumericConstant* /*randomNumberReplacement*/) {
    assert(false);
    return NULL;
}

LogicalExpression* AtomicLogicalExpression::determinizeMostLikely(NumericConstant* /*randomNumberReplacement*/) {
    return this;
}

LogicalExpression* NumericConstant::determinizeMostLikely(NumericConstant* /*randomNumberReplacement*/) {
    return this;
}

LogicalExpression* Conjunction::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new Conjunction(newExprs);
}

LogicalExpression* Disjunction::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new Disjunction(newExprs);
}

LogicalExpression* EqualsExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new EqualsExpression(newExprs);
}

LogicalExpression* GreaterExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new GreaterExpression(newExprs);
}

LogicalExpression* LowerExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new LowerExpression(newExprs);
}

LogicalExpression* GreaterEqualsExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new GreaterEqualsExpression(newExprs);
}

LogicalExpression* LowerEqualsExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new LowerEqualsExpression(newExprs);
}

LogicalExpression* Addition::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new Addition(newExprs);
}

LogicalExpression* Subtraction::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new Subtraction(newExprs);
}

LogicalExpression* Multiplication::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new Multiplication(newExprs);
}

LogicalExpression* Division::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newExprs;
    for(unsigned int i = 0; i < exprs.size(); ++i) {
        LogicalExpression* newExpr = exprs[i]->determinizeMostLikely(randomNumberReplacement);
        newExprs.push_back(newExpr);
    }
    return new Division(newExprs);
}

LogicalExpression* BernoulliDistribution::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    LogicalExpression* newExpr = expr->determinizeMostLikely(randomNumberReplacement);

    vector<LogicalExpression*> newExprs;
    newExprs.push_back(randomNumberReplacement);
    newExprs.push_back(newExpr);

    return new LowerEqualsExpression(newExprs);
}

LogicalExpression* IfThenElseExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    LogicalExpression* newCondition = condition->determinizeMostLikely(randomNumberReplacement);
    LogicalExpression* newValueIfTrue = valueIfTrue->determinizeMostLikely(randomNumberReplacement);
    LogicalExpression* newValueIfFalse = valueIfFalse->determinizeMostLikely(randomNumberReplacement);
    return new IfThenElseExpression(newCondition, newValueIfTrue, newValueIfFalse);
}

LogicalExpression* MultiConditionChecker::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    vector<LogicalExpression*> newConds;
    vector<LogicalExpression*> newEffs;
    for(unsigned int i = 0; i < conditions.size(); ++i) {
        LogicalExpression* newCond = conditions[i]->determinizeMostLikely(randomNumberReplacement);
        LogicalExpression* newEff = effects[i]->determinizeMostLikely(randomNumberReplacement);
        newConds.push_back(newCond);
        newEffs.push_back(newEff);
    }
    return new MultiConditionChecker(newConds, newEffs);
}

LogicalExpression* NegateExpression::determinizeMostLikely(NumericConstant* randomNumberReplacement) {
    LogicalExpression* newExpr = expr->determinizeMostLikely(randomNumberReplacement);
    return new NegateExpression(newExpr);
}


