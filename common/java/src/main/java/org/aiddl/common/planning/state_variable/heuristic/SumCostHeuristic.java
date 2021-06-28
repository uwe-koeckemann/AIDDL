package org.aiddl.common.planning.state_variable.heuristic;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.aiddl.common.planning.PlanningHeuristic;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.planning.state_variable.Operator;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Basic non-admissible heuristic that ignores negative preconditions and effects to compute cost to goal as sum
 * of reaching individual sub-goals (see Ghallab et al. Automated Planning Theory and Practice. Elsevier, 2014). 
 * 
 * @author Uwe Koeckemann
 *
 */
public class SumCostHeuristic implements Function, InterfaceImplementation, InitializableFunction, PlanningHeuristic {
	
	Set<Operator> O;
	SetTerm g;
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.heuristic");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	public void setOperators ( Set<Operator> O ) {
		this.O = O;
	}
	
	@Override
	public void initialize( Term args ) {
		SetTerm O = (SetTerm) args.get(PlanningTerm.Operators);
		SetTerm s = (SetTerm) args.get(PlanningTerm.InitialState);
		this.g = (SetTerm) args.get(PlanningTerm.Goal);
		
		Set<Operator> A = new LinkedHashSet<>();
		for ( Term o : O ) {
			A.add(new Operator((TupleTerm) o));
		}
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		
		Term O_ground = groundOps.compute(A, s);
		A.clear();
		for ( Term o : O_ground.asCollection() ) {
			A.add(new Operator(o.asTuple())); 
		}
		
		this.O = A;
		
	}
		
	@Override
	public Term apply(Term args) {
		return compute(args.asSet(), g);
	}
	

	public NumericalTerm compute( Set<Operator> O, SetTerm s, SetTerm g ) {
		this.O = O;
		return this.compute(s, g);
	}
	
	@Override
	public NumericalTerm compute(  SetTerm s, SetTerm g ) {
		Map<Term,Integer> delta_0 = new HashMap<>();
		Set<SetTerm> U = new LinkedHashSet<>();
		
		U.add(s);
		for ( Term p : s ) {
			delta_0.put(p, 0);
		}
		boolean change = true; 
		boolean found_goal = false;
		while ( change && ! found_goal ) {
			change = false;
			Set<SetTerm> new_U = new HashSet<>();
			Set<SetTerm> rem_U = new HashSet<>();
	
			for ( Operator a : O ) {
				SetTerm pre = a.getPreconditions();
				SetTerm eff = a.getEffects();
				
				boolean change_op = false;
				for ( SetTerm u : U ) {
					if ( u.containsAll(pre) && !u.containsAll(eff)) {
						for ( Term p : eff ) {
							Integer current = delta_0.get(p);
							int sum = 1;
							for ( Term q : pre ) {
								sum += delta_0.get(q);
							}
							if ( current == null || current > sum ) {
								delta_0.put( p, sum );
								change = true;
								change_op = true;
							}
						}
						
						SetTerm u_eff = u.addAll(eff);
								
						if ( u_eff.containsAll(g) ) {
							found_goal = true;;
							break;
						}
						new_U.add(u_eff);
						rem_U.add(u);
						if ( change_op )
							break;
					}
				}
				if ( found_goal ) {
					break;
				}
			}
			U.addAll(new_U);
			U.removeAll(rem_U);
		}
		
		int h = 0;
		for ( Term goal : g ) {
			if ( delta_0.containsKey(goal) ) {
				h += delta_0.get(goal);
			} else {
				return Term.infPos();
			}
		}
		return Term.integer(h);
	}
}
