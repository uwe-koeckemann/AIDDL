package org.aiddl.common.planning.state_variable.heuristic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.planning.PlanningHeuristic;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.planning.state_variable.Operator;
import org.aiddl.common.planning.state_variable.data.RelaxedPlanningGraphCreator;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Implementation of the fast forward heuristic.
 * 
 * @see Hoffmann, Joerg, & Nebel, B., The FF planning system: fast plan generation through heuristic search, Journal of Artificial Intelligence Research, 14(), 2001 (2001). 
 * 
 * @author Uwe Keckemann
 *
 */
public class FastForwardHeuristic implements Function, InterfaceImplementation, InitializableFunction, PlanningHeuristic {

	Map<SetTerm,ListTerm> rpgMap = new HashMap<>();
	Set<Operator> A = new HashSet<>();
	SetTerm g;
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.heuristic");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
		
	public void setActions ( Set<Operator> A ) {
		this.A = A; 
	}

	@Override
	public void initialize(Term args) {
		SetTerm O = (SetTerm) args.get(PlanningTerm.Operators);
		SetTerm s = (SetTerm) args.get(PlanningTerm.InitialState);
		this.g = (SetTerm) args.get(PlanningTerm.Goal);
		
		A = new LinkedHashSet<>();
		for ( Term o : O ) {
			A.add(new Operator((TupleTerm) o));
		}
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		
		Term O_ground = groundOps.compute(A, s);
		A.clear();
		for ( Term o : O_ground.asCollection() ) {
			A.add(new Operator(o.asTuple()));
		}
	}
	
	@Override
	public Term apply(Term args) {
		return compute ( args.asSet(), this.g );
	}
	
	@Override
	public NumericalTerm compute ( SetTerm s, SetTerm g ) {
		ListTerm relaxedPG = this.rpgMap.get(s);
		if ( s != null ) {
			RelaxedPlanningGraphCreator rpgCreator = new RelaxedPlanningGraphCreator();
			relaxedPG = rpgCreator.compute(A, s, g);
			this.rpgMap.put(s, relaxedPG);
		}
		
		Map<Term, List<Term>> achieverMap = new HashMap<>();
		Map<Term, Integer> actionDifficulty = new HashMap<>();
		Map<Term, Integer> earlierstLayer = new HashMap<>();
		
		if ( !relaxedPG.get(relaxedPG.size()-1).asSet().containsAll(g) ) {
			return Term.infPos();
		}
		for ( int i = 0 ; i < relaxedPG.size() ; i = i + 2  ) {
			for ( Term p : relaxedPG.get(i).asSet() ) {
				earlierstLayer.putIfAbsent(p, i);
			}
		}		
		for ( Term action : relaxedPG.get(relaxedPG.size()-2).asSet() ) {
			Operator a = new Operator(action.asTuple());
			for ( Term e : a.getEffects() ) {
				achieverMap.putIfAbsent(e, new ArrayList<>());
				achieverMap.get(e).add(action);
			}
			if ( a.getName().get(0).equals(Term.sym("NOOP")) ) {
				actionDifficulty.put(action, -1);
			} else {
				int difficulty = 0;
				for ( Term p : a.getPreconditions() ) {
					difficulty += earlierstLayer.get(p);
				}
				actionDifficulty.put(action, difficulty);
			}
		}
		
		Set<Term> unsatGoals = new HashSet<>();
		for ( Term goal : g.asSet() ) {
			Term g_sv = goal.getKey();
			Term g_a = goal.getValue();
			if ( s.asSet().containsKey(g_sv) ) {
				Term s_a = s.get(g_sv);
				if ( !s_a.equals(g_a) ) {
					unsatGoals.add(goal);
				}
			} else {
				unsatGoals.add(goal);
			}
		}
		Set<Term> selectedActions = new HashSet<>();
		
		for ( int i = relaxedPG.size()-1; i >= 2 ; i = i - 2 ) {
			Set<Term> newGoals = new HashSet<>();
			for ( Term goal : unsatGoals ) {
				Term selectedAction = null;
				Integer minCost = null;
				for ( Term action : relaxedPG.get(i-1).asSet() ) {
					if ( achieverMap.get(goal).contains(action) ) {
						if ( minCost == null || minCost > actionDifficulty.get(action) ) {
							minCost = actionDifficulty.get(action);
							selectedAction = action;
						}						
					}
				}
				selectedActions.add(selectedAction);
				Operator a = new Operator(selectedAction.asTuple());
				newGoals.addAll(a.getPreconditions().getLockedSet());
				
			}
			unsatGoals = newGoals;
		}
		return Term.integer(selectedActions.size());
	}
}
