package org.aiddl.common.planning;

import org.aiddl.common.learning.linear_regression.ExpansionFunction;
import org.aiddl.common.planning.state_variable.*;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.FunctionRegistry;

public class PlanningFunctionLoader {
    public static void register(FunctionRegistry F, Container C) {
        F.addFunction(PlanningUri.ITERATOR_FACTORY,
                new ForwardSearchPlanIterator());
        F.addFunction(PlanningUri.FORWARD_SEARCH,
                new ForwardSearchPlanner());
        F.addFunction(PlanningUri.APPLICABLE,
                new ApplicableFunction());
        F.addFunction(PlanningUri.GOAL_TEST,
                new GoalTest());
        F.addFunction(PlanningUri.TRANSITION,
                new StateTransitionFunction());
        F.addFunction(PlanningUri.EXPANSION,
                new ExpansionFunction());
        F.addFunction(PlanningUri.ENUM_REACHABLE,
                new OperatorReachableEnumerator());
        F.addFunction(PlanningUri.ENUM_STATE,
                new OperatorStateEnumerator());
        F.addFunction(PlanningUri.ENUM_CONSTRAINED_ACTIONS,
                new OperatorStateConstrainedDomainEnumerator());
        F.addFunction(PlanningUri.ENUM_DOMAIN_ACTIONS,
                new OperatorDomainEnumerator());
    }
}