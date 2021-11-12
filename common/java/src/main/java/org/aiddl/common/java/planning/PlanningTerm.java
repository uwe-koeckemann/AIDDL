package org.aiddl.common.java.planning;

import org.aiddl.core.java.representation.Term;

/**
 * Used to collect terms frequently used as keys.
 * 
 *
 * @author Uwe Koeckemann
 */
public class PlanningTerm {
	public static final Term Operators = Term.sym("operators");
	public static final Term InitialState = Term.sym("initial-state");
	public static final Term State = Term.sym("state");
	public static final Term Goal = Term.sym("goal");
	
	public static final Term Plan = Term.sym("plan");
	
	public static final Term Name = Term.sym("name");
	public static final Term Preconditions = Term.sym("preconditions");
	public static final Term Effects = Term.sym("effects");
	
	public static final Term Domains = Term.sym("domains");
	public static final Term Signature  = Term.sym("signature");
	public static final Term Signatures = Term.sym("signatures");
	
	public static final Term Heuristic = Term.sym("heuristic");
	
	public static final Term CausalGraph = Term.sym("causal-graph");
	public static final Term DomainTransitionGraphs = Term.sym("domain-transition-graphs");
	public static final Term PlanningGraph = Term.sym("planning-graph");
	public static final Term RelaxedPlanningGraph = Term.sym("relaxed-planning-graph");
		
	public static final Term Task = Term.sym("task");	
	public static final Term SubTasks = Term.sym("sub-tasks");	
	public static final Term OpenTasks = Term.sym("open-tasks");
	public static final Term Methods = Term.sym("methods");
	
	
	public static final Term NIL = Term.sym("NIL");
}
