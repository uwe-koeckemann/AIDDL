package org.aiddl.common.planning.state_variable;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class ApplicableFunction implements Function, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.applicable");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply(Term args) {
		Term a = args.get(0);
		SetTerm s = args.get(1).asSet();

		for ( Term p : a.get(PlanningTerm.Preconditions).asCollection() ) {
			Term p_sv = p.getKey();
			Term p_a = p.getValue();
			Term s_a = s.get(p_sv);
			if ( s_a == null || !s_a.equals(p_a) ) {
				return Term.bool(false);
			}
		}
		return Term.bool(true);
	}
}
