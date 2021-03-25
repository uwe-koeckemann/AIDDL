package org.aiddl.common.planning.state_variable;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.data.CausalGraphCreator;
import org.aiddl.common.planning.state_variable.data.DomainTransitionGraphCreator;
import org.aiddl.common.planning.state_variable.heuristic.CausalGraphHeuristic;
import org.aiddl.common.planning.state_variable.heuristic.FastForwardHeuristic;
import org.aiddl.common.planning.state_variable.heuristic.SumCostHeuristic;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;

public class HeuristicFactory {
	
	private static Term HeuristicCausalGraph = Term.sym("causal-graph");
	private static Term HeuristicAddCost = Term.sym("add-cost");
	private static Term HeuristicMaxCost = Term.sym("max-cost");
	private static Term HeuristicFastForward = Term.sym("fast-forward");

	public static Function createHeuristic( Term name, Term problem, SetTerm O_ground ) {
		if ( HeuristicCausalGraph.equals( name ) ) {
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
			h_cg.setGoal(problem.get(PlanningTerm.Goal).asSet());
			return h_cg;
		} else if ( HeuristicFastForward.equals( name ) ) {
			FastForwardHeuristic ffCalc = new FastForwardHeuristic();
			Set<Operator> A = new HashSet<>();
			for ( Term o : O_ground ) {
				A.add(new Operator(o.asTuple()));
			}
			ffCalc.setActions(A);
			return ffCalc;
		} else if ( HeuristicMaxCost.equals( name ) ) {
			SumCostHeuristic h_0 = new SumCostHeuristic();
			Set<Operator> A = new HashSet<>();
			for ( Term o : O_ground ) {
				A.add(new Operator(o.asTuple()));
			}
			h_0.setOperators(A);
			return h_0;
		} 
		throw new IllegalArgumentException("Unknown heuristic name: " + name);
	}
}
