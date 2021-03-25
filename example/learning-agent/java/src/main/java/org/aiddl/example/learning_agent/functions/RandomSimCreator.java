package org.aiddl.example.learning_agent.functions;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.ComboIterator;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;

public class RandomSimCreator implements ConfigurableFunction {

	private String name = RandomSimCreator.class.getSimpleName();
	boolean verbose = true;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}

	@Override
	public Term apply( Term args ) {
		Random r = new Random();
		int n = args.getIntValue();
		
		Set<Term> actions = new LinkedHashSet<>();
		
		List<List<Term>> choices = new ArrayList<List<Term>>();
		
		for ( int i = 0 ; i < n ; i++ ) {
			List<Term> stateChoice = new ArrayList<Term>();
			stateChoice.add(Term.sym("true"));
			stateChoice.add(Term.sym("false"));
			choices.add(stateChoice);
		}
		List<Term> state_transitions = new ArrayList<Term>();
		ComboIterator<Term> stateCombos = new ComboIterator<>(choices);
		for ( List<Term> combo : stateCombos ) {
			Set<Term> s_set = new HashSet<>();
			for ( int i = 1 ; i <= combo.size() ; i++ ) {
//				if ( combo.get(i-1).equals(Term.sym("true")) ) {
					s_set.add(
							Term.keyVal(Term.tuple(Term.sym("light"), Term.integer(i)),
									Term.bool(combo.get(i-1).equals(Term.sym("true")))));
//				}
			}
			SetTerm s = Term.set(s_set);
//			Map<Term, Term> s_sva = PlanningTerm.convert2svaMap(s);
			
			if ( verbose ) {
				Logger.msg(name, "State: " + s);
				Logger.incDepth();
			}
			
			Set<Term> transitions_for_s = new LinkedHashSet<>();
			for ( int j = 0 ; j < r.nextInt(3)+1 ; j++ ) {
				Set<Term> effects_set = new LinkedHashSet<Term>();
				Set<Term> s_next = new LinkedHashSet<>();
				
				int numChanges = r.nextInt(2)+1;
				Set<Integer> used = new HashSet<>();
				do {
					used.add(r.nextInt(n)+1);
				} while ( used.size() < numChanges ); 
				
				for ( int i = 1 ; i <= n ; i++ ) {

					Term variable = Term.tuple(Term.sym("light"), Term.integer(i));

					if ( used.contains(i) ) {
//						if ( s.contains(p_i) ) {
////							effects_set.add(Term.tuple(Term.sym("-"), p_i));
//							effects_set.add(Term.keyVal(p_i.getKey(), Term.bool(false)));
//						} else {
//							s_next.add(p_i);
//							effects_set.add(p_i);
//						}
						effects_set.add(Term.keyVal(variable, Term.bool(! s.get(variable).getBooleanValue() )));
						
						s_next.add(Term.keyVal(variable, Term.bool(! s.get(variable).getBooleanValue() )));								
//								Term.keyVal(variable, Term.bool(! s.get(variable).getBooleanValue() )));
					} else {
//						if ( s.contains(p_i) ) {
							s_next.add(Term.keyVal(variable, s.get(variable)));
//						}
					}
				}
				 
				SetTerm effects = Term.set(effects_set);
				
				if ( effects_set.isEmpty() || transitions_for_s.contains(effects) ) {
					j--;
					continue;
				}
//				transitions_for_s.add(effects);
//				Term actionName = Term.tuple(Term.sym("op"), effects);
//				Term actionName = Term.tuple(Term.sym("op"), Term.integer(effects.hashCode()));
				Term actionName = Term.tuple(Term.sym("push"), Term.integer(r.nextInt(n) +1));
				actions.add(actionName);
				state_transitions.add(Term.keyVal(Term.tuple(actionName, s), Term.set(s_next)));
				
				if ( verbose ) Logger.msg(name, "Action: " + actionName);
				if ( verbose ) Logger.msg(name, "Next state: " + s_next);
			}
			Logger.decDepth();
		}
		
		if ( verbose ) {
			Logger.msg(name, "Transitions: "  + state_transitions.size());
			Logger.msg(name, "Actions:"  + actions.size());
		}

		LockableList rList = new LockableList();
		rList.add(Term.keyVal(TransKey, Term.list(state_transitions)));
		rList.add(Term.keyVal(ActionsKey, Term.set(actions)));
		return Term.list(rList);
	}
	
	private static final Term TransKey = Term.sym("state-transitions");
	private static final Term ActionsKey = Term.sym("actions");
}
