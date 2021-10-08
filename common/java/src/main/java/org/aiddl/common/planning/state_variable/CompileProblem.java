package org.aiddl.common.planning.state_variable;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;

/**
 * Compile a planning problem to an integer representation to make the solver more efficient.
 * 
 * @author Uwe
 *
 */
public class CompileProblem implements Function {
	
	private int nextID;

	@Override
	public Term apply(Term problem) {
		SetTerm s0 = problem.get(0).asSet();
		SetTerm g = problem.get(1).asSet();
		SetTerm O = problem.get(2).asSet();		
	
		Map<Term,Term> valueMap = new HashMap<>();
		this.nextID = 1;
		
		SetTerm s0_new = doSet(s0, valueMap);
		SetTerm g_new = doSet(g, valueMap);
		LockableSet O_new = new LockableSet();
			
		for ( Term o : O ) {
			Term name = o.get(PlanningTerm.Name);
			Term name_new = valueMap.get(name);
			if ( name_new == null ) {
				name_new = Term.integer(nextID);
				valueMap.put(name, name_new);
				nextID++;
			}
			Term pre_new = doSet(o.get(PlanningTerm.Preconditions).asSet(), valueMap);
			Term eff_new = doSet(o.get(PlanningTerm.Effects).asSet(), valueMap);
			
			O_new.add(Term.tuple(
					Term.keyVal(PlanningTerm.Name, name_new),
					Term.keyVal(PlanningTerm.Preconditions, pre_new),
					Term.keyVal(PlanningTerm.Effects, eff_new)));
		}
		
		LockableSet map = new LockableSet();
		for ( Term k : valueMap.keySet() ) {
			map.add(Term.keyVal(valueMap.get(k), k));
		}
		
		return Term.tuple(
				s0_new,	
				g_new,
				Term.set(O_new),
				Term.set(map));
	}
	
	private SetTerm doSet( SetTerm S, Map<Term,Term> valueMap ) {
		LockableSet S_new = new LockableSet();
		
		for ( Term e : S ) {
			Term k = e.getKey();
			Term v = e.getValue();
			
			Term k_new = valueMap.get(k);
			if ( k_new == null ) {
				k_new = Term.integer(nextID);
				valueMap.put(k, k_new);
				nextID++;
			}
			Term v_new = valueMap.get(v);
			if ( v_new == null ) {
				v_new = Term.integer(nextID);
				valueMap.put(v, v_new);
				nextID++;
			}
			S_new.add(Term.keyVal(k_new, v_new));
		}
		return Term.set(S_new);
	}

}
