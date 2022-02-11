package org.aiddl.common.java.planning;

import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class PlanningUri {
    public static SymbolicTerm ITERATOR_FACTORY= Term.sym("org.aiddl.common.planning.state-variable.plan-iterator-factory");
    public static SymbolicTerm FORWARD_SEARCH = Term.sym("org.aiddl.common.planning.state-variable.forward-search-planner");
    public static SymbolicTerm APPLICABLE = Term.sym("org.aiddl.common.planning.state-variable.applicable");
    public static SymbolicTerm GOAL_TEST = Term.sym("org.aiddl.common.planning.state-variable.goal-test");
    public static SymbolicTerm TRANSITION = Term.sym("org.aiddl.common.planning.state-variable.state-transition");
    public static SymbolicTerm EXPANSION = Term.sym("org.aiddl.common.planning.state-variable.expand");
    public static SymbolicTerm ENUM_REACHABLE = Term.sym("org.aiddl.common.planning.state-variable.enum-reachable-actions");
    public static SymbolicTerm ENUM_STATE = Term.sym("org.aiddl.common.planning.state-variable.enum-state-actions");
    public static SymbolicTerm ENUM_CONSTRAINED_ACTIONS = Term.sym("org.aiddl.common.planning.state-variable.enum-constrained-actions");
    public static SymbolicTerm ENUM_DOMAIN_ACTIONS = Term.sym("org.aiddl.common.planning.state-variable.enum-domain-actions");
}
