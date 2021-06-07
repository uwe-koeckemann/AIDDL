package org.aiddl.common.planning.state_variable;

import java.util.Map;
import org.aiddl.common.CommonTerm;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.search.GraphSearch;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.Logger;

public class ForwardSearchPlanIterator implements InitializableFunction, ConfigurableFunction {
	
	FunctionRegistry fReg;
	GraphSearch search;
	
	SymbolicTerm heuristicName = Term.sym("causal-graph");
	
	String name = "PlanIterator";
	int verbose = 0;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg;
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		this.heuristicName = settings.getOrDefault(PlanningTerm.Heuristic, this.heuristicName).asSym();
	}
	
	@Override
	public void initialize(Term problem) {
		SetTerm O = (SetTerm) problem.get(PlanningTerm.Operators);
		SetTerm s0 = problem.get(PlanningTerm.InitialState).asSet();
		SetTerm g = problem.get(PlanningTerm.Goal).asSet();
		
//		LockablSet<Operator> A = new LinkedHashSet<>();
//		for ( Term o : O ) {
//			A.add(new Operator((TupleTerm) o));
//		}
		if ( verbose >= 1 ) {
			Logger.msg(name, "Initial State: " + s0);
			Logger.msg(name, "Goal         : " + g);
		}
	
		GoalTest goalTest = new GoalTest();
		
		goalTest.initialize(g);

		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		
		Term O_ground = groundOps.apply(problem);
		
//		System.out.println("Applicable ops: " + O_ground.size());
		
		Function h = HeuristicFactory.createHeuristic(this.heuristicName, problem, O_ground.asSet());
		
		Expand expand = new Expand();
		expand.initialize(O_ground);
				
		search = new GraphSearch();
		search.setExpansion(expand);
		search.setGoalTest(goalTest);
		search.setHeuristic(h);
		search.setSearchParameters(Term.rational(5, 10), true);
		search.initialize(Term.list(s0));
	}
	
	Term last_solution;
	boolean done = false;
	
	@Override
	public Term apply(Term args) {
		if ( args.equals(Term.sym("next")) ) {
			if ( done ) {
				return CommonTerm.NIL;
			}
			Term next_goal_node = search.apply(Term.tuple(GraphSearch.Search));
			if ( next_goal_node.equals(CommonTerm.NIL) ) {
				done = true;
				return next_goal_node;
			}
//			System.out.println(next_goal_node);
			Term path = search.apply(Term.tuple(GraphSearch.Get, GraphSearch.Path, next_goal_node));
			this.last_solution = path;
			
			return path;
		}
		return CommonTerm.NIL;
	}
}
