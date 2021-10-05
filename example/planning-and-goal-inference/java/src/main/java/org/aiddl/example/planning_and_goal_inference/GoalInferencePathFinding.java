package org.aiddl.example.planning_and_goal_inference;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.math.graph.BellmanFord;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.reasoning.logic.LogicTerm;
import org.aiddl.core.function.Uri;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;

public class GoalInferencePathFinding implements ConfigurableFunction {
	
	int verbose = 0;
	String name = "GoalInference";	
	
	Function eval;
	FunctionRegistry freg;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		this.eval = fReg.getFunction(settings.getOrDefault(Term.sym("eval"), Uri.EVAL));
		this.freg = fReg;
	}

	@Override
	public Term apply( Term args ) {
		SetTerm g = args.get(PlanningTerm.Goal).asSet();
		SetTerm Q = args.get(Term.sym("query")).asSet();
		TupleTerm O = args.get(Term.sym("ontology")).asTuple();
		
		SetTerm new_goals = g;
		
		boolean sat = true;
		Set<Term> inferredGoals = new HashSet<>();
		
		for ( Term q : Q ) {
			if ( verbose >= 1 ) {
				Logger.msg(name, "Query: " + q);
				Logger.incDepth();
			}
			
			Term goal = q.get(0);
			Term sig  = q.get(1);
			
			boolean isKeyValuePair = (goal instanceof KeyValueTerm);
			TupleTerm t_goal = null;
			TupleTerm t_sig = null;
			if ( isKeyValuePair ) { 
				t_goal = goal.getKey().asTuple();
				t_sig = sig.getKey().asTuple();
			} else {
				t_goal = goal.asTuple();
				t_sig= sig.asTuple();
			}
			boolean goal_reachable = true;
			
			List<Term> tuple_result = new ArrayList<>();
			for ( int i = 0 ; i < t_goal.size() ; i++ ) {
				Term r = infer( t_goal.get(i), t_sig.get(i), O);
				if ( r != null ) {
					tuple_result.add(r);
				} else {
					if ( verbose >= 1 ) Logger.msg(name, "Goal unreachable: " + goal);
					goal_reachable = false;
					sat = false;
					break;
				}
			}
			
			Term result = null;
			if ( goal_reachable && isKeyValuePair ) {
				Term r = infer( goal.getValue(), sig.getValue(), O);
				if ( r != null ) {
					result = Term.keyVal(Term.tuple(tuple_result), r);
					tuple_result.add(r);
				} else {
					if ( verbose >= 1 ) Logger.msg(name, "Goal unreachable: " + goal);
					goal_reachable = false;
					sat = false;
					break;
				}
				
			} 
			
			if ( goal_reachable && !isKeyValuePair ) {
				result = Term.tuple(tuple_result);
			}
			
			if ( verbose >= 1 ) {
				Logger.decDepth();
				Logger.msg(name, "Usable goal: " + result);
			}
			if ( result != null ) {
				inferredGoals.add(result);
				new_goals = (SetTerm) new_goals.remove(goal).add(result);
			}
		}
		
		if ( verbose >= 1 ) {
			Logger.msg(name, "New set of goals: " + new_goals);
		}
		LockableList rList = new LockableList();
		rList.add(Term.keyVal(LogicTerm.SAT, Term.bool(sat)));
		rList.add(Term.keyVal(PlanningTerm.Goal, new_goals));
		return Term.list(rList);
	}
		
	private Term infer( Term g, Term s, TupleTerm ontology ) {
		if ( g.equals(s) ) {
			return g;
		} else {
			if ( verbose >= 1 ) {
				Logger.msg(name, "Looking for path from \"" + g + "\" to one of " + s);
				Logger.incDepth();
			}
			
			Term f_weight_lambda = this.eval.apply(Parser.ParseTerm("(org.aiddl.eval.lambda ?x 1)"));
			Term f_weight_ref = eval.apply(f_weight_lambda);
			
			TupleTerm args = Term.tuple(
					ontology,
					f_weight_ref,
					g);
			BellmanFord bf = new BellmanFord();
			
			Term r = bf.apply(args);
			
			Term argMin = null;
			NumericalTerm minPathLength = Term.infPos();
			Term dMap = r.get(Term.sym("distance"));			
			for ( Term t : s.asCollection() ) {
				NumericalTerm len = (NumericalTerm) dMap.get(t);
				if ( len.lessThan(minPathLength) ) {
					Logger.msg(name, t + " is a " + g +  " (|path|="  + dMap.get(t) + ")");
					minPathLength = len;
					argMin = t;
				}
			}
			if ( verbose >= 1 ) {
				Logger.decDepth();
			}
			return argMin;
		}
	}
}
	
