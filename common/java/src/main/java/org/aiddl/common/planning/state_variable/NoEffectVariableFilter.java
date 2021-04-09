package org.aiddl.common.planning.state_variable;

import java.util.HashSet;
import java.util.Set;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;

/**
 * Take a state, a goal, and a set of ground actions and remove all occurrences of state-variables 
 * that never change through effects.
 * 
 * @author Uwe Koeckemann
 *
 */
public class NoEffectVariableFilter implements Function {

	@Override
	public Term apply(Term args) {
		SetTerm s = args.get(0).asSet();
		SetTerm g = args.get(1).asSet();
		SetTerm A = args.get(2).asSet();
		
		Set<Term> changingStateVariables = new HashSet<>();
		
		for ( Term a : A ) {
			for ( Term e : a.get(PlanningTerm.Effects).asSet() ) {
				changingStateVariables.add(e.getKey());
			}
		}
		
		LockableSet s_new = new LockableSet();
		LockableSet g_new = new LockableSet();
		LockableSet A_new = new LockableSet();
		
		for ( Term sva : s ) {
			if ( changingStateVariables.contains(sva.getKey()) ) {
				s_new.add(sva);
			}
		}
		for ( Term sva : g ) {
			if ( changingStateVariables.contains(sva.getKey()) ) {
				g_new.add(sva);
			} else {
				if ( !s.contains(sva) ) {
//					Logger.msg("NoEffectVariableFilter", "Goal " + sva + " unreachable: depends on constant variable and is not in initial state.");
					g_new.add(sva); // Keep unreachable goals
				}
			}
		}
		
		for ( Term a : A ) {
			LockableSet P_new = new LockableSet();
			for ( Term p : a.get(PlanningTerm.Preconditions).asSet() ) {
				if ( changingStateVariables.contains(p.getKey())) {
					P_new.add(p);
				}
			}
			LockableList new_a = new LockableList();
			for ( int i = 0 ; i < a.size() ; i++ ) {
				if ( !a.get(i).getKey().equals(PlanningTerm.Preconditions) ) {
					new_a.add(a.get(i));
				} else {
					new_a.add(Term.keyVal(PlanningTerm.Preconditions, Term.set(P_new)));
				}
			}
			A_new.add(Term.tuple(new_a));
		}
		
//		Logger.msg("NoEffectVariableFilter", "New state: " + Term.set(s_new));
//		Logger.msg("NoEffectVariableFilter", "New goal: " + Term.set(g_new));
//		for ( Term a : Term.set(A_new) ) {
//			Logger.msg("NoEffectVariableFilter", "--- Action ---\n " + Logger.prettyPrint(a, 1));
//		}
		
		return Term.tuple(Term.set(s_new), Term.set(g_new), Term.set(A_new));
	}
	
	

}
