package org.aiddl.common.planning.state_variable;

import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.planning.PlanningHeuristic;
import org.aiddl.common.planning.state_variable.data.CausalGraphCreator;
import org.aiddl.common.planning.state_variable.data.DomainTransitionGraphCreator;
import org.aiddl.common.planning.state_variable.heuristic.CausalGraphHeuristic;
import org.aiddl.common.planning.state_variable.heuristic.FastForwardHeuristic;
import org.aiddl.common.planning.state_variable.heuristic.SumCostHeuristic;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.tools.StopWatch;

import java.util.Set;

public class ForwardSearchPlanner implements Function, ConfigurableFunction, InterfaceImplementation {		
	String name = ForwardSearchPlanner.class.getSimpleName();
	int verbose = 0;
	
	private PlanningHeuristic heuristic;
	private Term heuristic_name = Term.sym("causal-graph");
	
	private Term HeuristicCausalGraph = Term.sym("causal-graph");
	private Term HeuristicAddCost = Term.sym("add-cost");
	private Term HeuristicMaxCost = Term.sym("max-cost");
	private Term HeuristicFastForward = Term.sym("fast-forward");
	
	private GoalTest goalTest = new GoalTest();
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.planner");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}


	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		this.heuristic_name = settings.getOrDefault(PlanningTerm.Heuristic, this.heuristic_name);
	}
	
	@Override
	public Term apply( Term problem ) {			
		SetTerm O = (SetTerm) problem.get(PlanningTerm.Operators);
		SetTerm s0 = problem.get(PlanningTerm.InitialState).asSet();
		SetTerm g = problem.get(PlanningTerm.Goal).asSet();
		
		Set<Operator> A = new LinkedHashSet<>();
		for ( Term o : O ) {
			A.add(new Operator((TupleTerm) o));
		}
		if ( verbose > 0 ) {
			Logger.msg(name, "Initial State: " + s0);
			Logger.msg(name, "Goal         : " + g);
		}
		goalTest.initialize(g);

		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		
		Term O_ground = groundOps.compute(A, s0);
		A.clear();
		for ( Term o : O_ground.asCollection() ) {
			A.add(new Operator(o.asTuple())); 
		}
		
		if ( HeuristicCausalGraph.equals( this.heuristic_name ) ) {
			CausalGraphCreator causalGraphComp = new CausalGraphCreator();
			DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
			
			Graph CG = new Graph( causalGraphComp.apply(O_ground) );
			
			Term DTGs_term = dtgComp.apply(O_ground);
			Map<Term, Graph> DTGs = new HashMap<>();
			for ( Term entry : DTGs_term.asCollection() ) {
				DTGs.put(entry.getKey(), new Graph(entry.getValue()));
			}
			
			CausalGraphHeuristic h_cg = new CausalGraphHeuristic();
			h_cg.setDataStructures(CG, DTGs);
			this.heuristic = h_cg;
		} else if ( HeuristicFastForward.equals( this.heuristic_name ) ) {
			FastForwardHeuristic ffCalc = new FastForwardHeuristic();
			ffCalc.setActions(A);
			this.heuristic = ffCalc;
		} else if ( HeuristicMaxCost.equals( this.heuristic_name ) ) {
			SumCostHeuristic h_0 = new SumCostHeuristic();
			h_0.setOperators(A);
			this.heuristic = h_0;
		} else {
			throw new IllegalArgumentException("Unknown heuristic name: " + heuristic_name);
		}
		
		if ( verbose >= 1 ) {
			Logger.msg(name, "Operators: "+O_ground.size());
			if ( verbose >= 2) {
				Logger.incDepth();
				for ( Term a : O_ground.asCollection() ) {
					Logger.msg(name, "  " + a.toString());
				}
				Logger.decDepth();
			}
		}
		
		Set<SetTerm> visited = new LinkedHashSet<>();
		visited.add(s0);
			
		List<Term> plan = plan(A, s0, g, new LinkedList<Term>(), visited);

		if ( plan != null ) {
			if ( verbose >= 1 ) Logger.msg(name, "|plan| = " + plan.size() + " -> " + plan.toString());
			return Term.list(plan); //Term.keyVal(PlanningTerm.Plan, Term.list(plan));
		} else {
			if ( verbose >= 1 ) Logger.msg(name, "No plan found.");
			return Term.sym("NIL"); //Term.keyVal(PlanningTerm.Plan, Term.sym("NIL"));
		}
	}


	
	private List<Term> plan( Set<Operator> O, SetTerm s, SetTerm g, List<Term> pi, Set<SetTerm> visited ) {
		if ( satisfied(s, g)) {
			return pi;
		}
		PriorityQueue<StateActionHeuristic> Q = new PriorityQueue<>();
		if ( verbose >= 3 ) {
			Logger.msg(name, "Checking applicable...");
			Logger.incDepth();
		}
		
		for ( Operator a : applicable(O, s) ) {
			SetTerm s_succ = apply(a, s);
			if ( !visited.contains(s_succ) ) {
				StopWatch.start("h");
				
				NumericalTerm h = this.heuristic.compute(s_succ, g);	
				StopWatch.stop("h");
				if ( !h.isInfPos() ) {
					Q.add(new StateActionHeuristic(s_succ, a, h.getIntValue()));
					if ( verbose >= 3 ) Logger.msg(name, "h=" + h + " a=" + a + " s'=" + s_succ + " g=" + g);
					
				}
			}
		}
		if ( verbose >= 3 ) {
			Logger.decDepth();
			Logger.msg(name, "Found: " + Q.size() + " applicable... visited: " + visited.size());
		}
		while ( !Q.isEmpty() ) {
			StateActionHeuristic sah = Q.poll();
			SetTerm s_succ = sah.s;
			Operator a = sah.a;
			if ( !visited.contains(s_succ) ) {
				visited.add(s_succ);
				List<Term> pi_next = new LinkedList<Term>();
				pi_next.addAll(pi);
				pi_next.add(a.getName());
				List<Term> pi_sol = plan(O, s_succ, g, pi_next, visited);
				if ( pi_sol != null ) {
					return pi_sol;
				}
			}
		}
		return null;
	}
	
	class StateActionHeuristic implements Comparable<StateActionHeuristic> {
		public SetTerm s;
		public Operator a; 
		public int h;
		
		public StateActionHeuristic( SetTerm s, Operator a, int h ) {
			this.s = s;
			this.a = a;
			this.h = h;
		}
		
		@Override
		public int compareTo(StateActionHeuristic o) {
			return this.h-o.h;
		}
		
	}

	private SetTerm apply( Operator a, SetTerm s ) {
		return s.putAll(a.getEffects());
	}
	
	private Set<Operator> applicable( Set<Operator> O, SetTerm s ) {
		Set<Operator> applicableOps = new LinkedHashSet<>();
		for(  Operator o : O ) {

			boolean applicable = true;
			for ( Term pre : o.getPreconditions() ) {
				Term s_a = s.get(pre.getKey());
				
				if ( s_a == null || !s_a.equals(pre.getValue()) ) {
					applicable = false;
					break;
				}
			}
			if ( applicable ) {
				applicableOps.add(o);
			}
		}
		return applicableOps;
	}
	
	private boolean satisfied( SetTerm s, SetTerm g ) {
//		System.out.println(s + " SAT \n\t" + g + "\n\t" +  goalTest.compute(s).getBooleanValue());
		return goalTest.apply(s).getBooleanValue();
	}
}
