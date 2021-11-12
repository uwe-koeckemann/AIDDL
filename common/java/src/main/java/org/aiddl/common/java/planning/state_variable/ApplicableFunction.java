package org.aiddl.common.java.planning.state_variable;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

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
