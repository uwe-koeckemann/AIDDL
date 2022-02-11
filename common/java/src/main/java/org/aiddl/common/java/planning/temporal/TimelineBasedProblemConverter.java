package org.aiddl.common.java.planning.temporal;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableSet;

public class TimelineBasedProblemConverter implements Function {

	@Override
	public Term apply(Term args) {
		SetTerm O = args.get(PlanningTerm.Operators).asSet();
		SetTerm s = args.get(PlanningTerm.InitialState).asSet();
		SetTerm g = args.get(PlanningTerm.Goal).asSet();
		Term domains = args.get(PlanningTerm.Domains);
		
		LockableSet s_new = new LockableSet();
		for ( Term statement : s ) {
			s_new.add(Term.keyVal(statement.get(1), statement.get(2)));
		}
		
		LockableSet g_new = new LockableSet();
		for ( Term statement : g ) {
			g_new.add(Term.keyVal(statement.get(1), statement.get(2)));
		}		
		
		LockableSet O_sva = new LockableSet();
		for ( Term o : O ) {
			Term name = o.get(PlanningTerm.Name);
			SetTerm pre = o.get(PlanningTerm.Preconditions).asSet();
			SetTerm eff = o.get(PlanningTerm.Effects).asSet();
			
			LockableSet pre_new = new LockableSet();
			for ( Term p : pre ) {
				pre_new.add(Term.keyVal(p.get(1), p.get(2)));
			}
			LockableSet eff_new = new LockableSet();
			for ( Term e : eff ) {
				eff_new.add(Term.keyVal(e.get(1), e.get(2)));
			}
			
			Term signature = o.get(PlanningTerm.Signature);
			
			if ( signature == null ) {
				O_sva.add(Term.tuple(
					Term.keyVal(PlanningTerm.Name, name),
					Term.keyVal(PlanningTerm.Preconditions, Term.set(pre_new)),
					Term.keyVal(PlanningTerm.Effects, Term.set(eff_new)))
					);
			} else {
				O_sva.add(Term.tuple(
					Term.keyVal(PlanningTerm.Name, name),
					Term.keyVal(PlanningTerm.Signature, signature),
					Term.keyVal(PlanningTerm.Preconditions, Term.set(pre_new)),
					Term.keyVal(PlanningTerm.Effects, Term.set(eff_new)))
					);
			}
		}
		
		if ( domains == null ) {
			return Term.tuple(
					Term.keyVal(PlanningTerm.InitialState, Term.set(s_new)),
					Term.keyVal(PlanningTerm.Goal, Term.set(g_new)),
					Term.keyVal(PlanningTerm.Operators, Term.set(O_sva)));
		} else {
			return Term.tuple(
					Term.keyVal(PlanningTerm.InitialState, Term.set(s_new)),
					Term.keyVal(PlanningTerm.Goal, Term.set(g_new)),
					Term.keyVal(PlanningTerm.Domains, domains),
					Term.keyVal(PlanningTerm.Operators, Term.set(O_sva)));
		}
	}

}
