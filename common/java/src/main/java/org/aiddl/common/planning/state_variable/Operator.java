package org.aiddl.common.planning.state_variable;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

public class Operator {
	Term o;
	
//	Map<Term, Term> p_svaMap = new HashMap<>();
//	Map<Term, Term> e_svaMap = new HashMap<>();
	
//	SetTerm preconditions = null;
//	SetTerm effects = null;
	
	public Operator( TupleTerm o ) {
		this.o = o;
//		for ( Term p : this.o.get(PlanningTerm.Preconditions).asSet() ) {
//			p_svaMap.put(p.get(PlanningTerm.StateVariable), p.get(PlanningTerm.Assignment));
//		}
//		for ( Term e : this.o.get(PlanningTerm.Effects).asSet() ) {
//			e_svaMap.put(e.get(PlanningTerm.StateVariable), e.get(PlanningTerm.Assignment));
//		}
	}
	public Term getOperatorTerm() {
		return o;
	}
	public Term getName() {
		return o.get(PlanningTerm.Name);
	}
//	public Map<Term,Term> getPreMap( ) {
//		return this.p_svaMap;
//	}
//	public Map<Term,Term> getEffMap( ) {
//		return this.e_svaMap;
//	}
	public SetTerm getPreconditions() {
//		if ( preconditions == null ) {
//			this.preconditions = PlanningTerm.convertSVA(o.get(PlanningTerm.Preconditions).asSet());
//		}
//		return this.preconditions; //
		return (SetTerm) o.get(PlanningTerm.Preconditions);
	}
	public SetTerm getEffects() {
//		if ( effects == null ) {
//			this.effects = PlanningTerm.convertSVA(o.get(PlanningTerm.Effects).asSet());
//		}
//		return this.effects; // 
		return (SetTerm) o.get(PlanningTerm.Effects);
	}	
	public SetTerm applyTo( SetTerm s ) {
//		HashMap<Term, Term> svaMapFull = new HashMap<>();
//		
//		for ( Term s_e : s ) {
//			svaMapFull.put(s_e.get(PlanningTerm.StateVariable), s_e);
//		}
//		
//		LockableSet s_succ = s.getLockedSet().unlock();
//		for ( Term e : this.getEffects() ) {
//			s_succ.remove(svaMapFull.get(e.get(PlanningTerm.StateVariable)));
//			s_succ.add(e);
//		}
		return s.putAll(this.getEffects()); // Term.set(s_succ);
	}
//	public boolean applicable( SetTerm s ) {
//		return s.containsAll(this.getPreconditions());
//	}
	public Operator substitute( Substitution s ) {
		return new Operator(this.o.substitute(s).asTuple());
	}
	/**
	 * Assemble an operator term from name, preconditions, and effects
	 * @param name name of the operator
	 * @param precondition condition that must be satisfied to apply operator
	 * @param effects changes in state when operator is applied
	 * @return assembled operators
	 */
	public static Term assembleOperator( Term name, CollectionTerm precondition, CollectionTerm effects ) {
		return Term.tuple(
				Term.keyVal(PlanningTerm.Name, name),
				Term.keyVal(PlanningTerm.Preconditions,	precondition),
				Term.keyVal(PlanningTerm.Effects, effects));
		
	}
	
	@Override 
	public int hashCode() {
		return this.o.hashCode() * 21;
	}
	
	@Override 
	public boolean equals( Object o ) {
		if ( !(o instanceof Operator)) {
			return false;
		}
		Operator a = (Operator)o;
		return this.o.equals(a.o);
	}
	
	@Override 
	public String toString() {
		return o.toString();
	}
}
