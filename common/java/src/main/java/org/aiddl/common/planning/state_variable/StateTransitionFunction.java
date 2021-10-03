package org.aiddl.common.planning.state_variable;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class StateTransitionFunction implements Function, InterfaceImplementation {

	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.apply");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply(Term args) {
		Term s = args.get(0);
		Term a = args.get(1);
		return s.asSet().putAll(a.get(PlanningTerm.Effects).asSet());
	}
}
