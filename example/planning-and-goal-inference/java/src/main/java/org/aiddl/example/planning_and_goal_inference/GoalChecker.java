package org.aiddl.example.planning_and_goal_inference;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;

public class GoalChecker implements ConfigurableFunction {
	
	int verbose = 0;
	String name = "GoalChecker";	
	
	Function eval;
	
	private static Term EvalDomain = Term.sym("org.aiddl.eval.domain"); 
			
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		this.eval = fReg.getFunction(settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL));
	}
		
	@Override
	public Term apply( Term args ) {
		SetTerm g = args.get(PlanningTerm.Goal).asSet();
		SetTerm S = args.get(PlanningTerm.Signatures).asSet();
		
		Set<Term> unsatGoals = new HashSet<>();
		
		for ( Term goal : g ) {
			if ( verbose >= 1 ) {
				Logger.msg(name, "Checking domain for: " + goal);
				Logger.incDepth();
			}
			Term goodSig = null;
			
			for ( Term s : S.asCollection() ) {
				
				if ( s instanceof TupleTerm && goal instanceof TupleTerm ) {
					Term a = this.checkTuple(s, goal);
					if ( a != null ) {
						goodSig = a;
						break;
					}
				} else if ( s instanceof KeyValueTerm && goal instanceof KeyValueTerm ) {
					Term a = this.checkKeyValuePair(s, goal);
					if ( a != null ) {
						goodSig = a;
						break;
					}
				}
				
			}
			if ( goodSig != null ) {
				if ( !goal.equals(goodSig) ) {
					Term query = Term.tuple(goal, goodSig);
					unsatGoals.add(query);
					if ( verbose >= 1 ) {
						Logger.msg(name, "need inference to get from \"" + goal + "\" to \"" + goodSig + "\"");
					}
				} else {
					if ( verbose >= 1 ) {
						Logger.msg(name, "nothing to do");
					}
				}
			} 
			if ( verbose >= 1 ) {
				Logger.decDepth();
			}
		}

		return Term.set(unsatGoals);
	}
		
	private Term checkKeyValuePair ( Term s, Term goal ) {
		Term k = this.checkTuple(s.getKey(), goal.getKey() );
		if ( k != null ) {
			Term v = null; 
			Term arg = Term.tuple(EvalDomain, s.getValue());
			Term D = eval.apply(arg);
			if ( D.equals(goal.getValue()) 
					|| (  D instanceof CollectionTerm && D.asCollection().contains(goal.getValue())) ) {
				v = goal.getValue();
			} else if ( D instanceof CollectionTerm && !D.asCollection().contains(goal.getValue()) ) {
				v = D;
			} else {
				return null;
			}
			return Term.keyVal(k, v);
		}
		return null;
	}
		
		
	private Term checkTuple ( Term s, Term goal ) {
		if ( goal.size() == s.size() ) {
			List<Term> query = new ArrayList<>();
			for ( int i = 0 ; i < s.size() ; i++ ) {
				Term arg = Term.tuple(EvalDomain, s.get(i));
				Term D = eval.apply(arg);
				
				if ( D.equals(goal.get(i)) 
				|| (  D instanceof CollectionTerm && D.asCollection().contains(goal.get(i))) ) {
					query.add(goal.get(i));
				} else if ( D instanceof CollectionTerm && !D.asCollection().contains(goal.get(i)) ) {
					query.add(D);
				} else {
					return null;
				}
			}
			return Term.tuple(query);
		}
		return null;
	}
}