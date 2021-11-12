package org.aiddl.example.planning_and_goal_inference;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.common.java.reasoning.logic.LogicTerm;
import org.aiddl.common.java.reasoning.logic.prolog.PrologQueryRunner;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.KeyValueTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.Logger;

public class GoalInferenceProlog implements ConfigurableFunction {
	
	int verbose = 0;
	String name = "GoalInference";	
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}
	
	@Override
	public Term apply( Term args ) {
		SetTerm g = args.get(PlanningTerm.Goal).asSet();
		SetTerm Q = args.get(Term.sym("query")).asSet();
		Term KB = args.get(Term.sym("kb"));
		
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
				Term r = infer( t_goal.get(i), t_sig.get(i), KB);
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
				Term r = infer( goal.getValue(), sig.getValue(), KB);
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
		
	private Term infer( Term g, Term s, Term kb ) {
		if ( g.equals(s) ) {
			return g;
		} else {
			if ( verbose >= 1 ) {
				System.out.println(g);
				Logger.msg(name, "Looking for subclass of \"" + g + "\" to be one of " + s);
				Logger.incDepth();
			}
			Term variable = Term.var("X");
			
			TupleTerm args = Term.tuple(
					Term.keyVal(LogicTerm.Query, Term.list(Term.tuple(Term.sym("subclass_of"), g, variable))),
					Term.keyVal(LogicTerm.KB, kb));
			PrologQueryRunner prologQueryRunner = new PrologQueryRunner();
			
			
			
			Term answer = prologQueryRunner.apply(args);
			
			if ( answer.equals(CommonTerm.NIL) ) {
				if ( verbose >= 1 ) {
					Logger.msg(name, "No solution!");
					Logger.decDepth();
				}
				return answer;
			}
			
			for ( Term sub : answer.asList() ) {
				Term value = sub.get(variable);
				 if ( s.asCollection().contains(value) ) {
					 if ( verbose >= 1 ) {
						 Logger.msg(name, "Working goal symbol: " + value);
						 Logger.decDepth();
					 }
					 return value;
				 }
			}

			if ( verbose >= 1 ) {
				Logger.decDepth();
			}
			return CommonTerm.NIL;
		}
	}
}
	
