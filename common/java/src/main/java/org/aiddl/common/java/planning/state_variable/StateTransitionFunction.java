package org.aiddl.common.java.planning.state_variable;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

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
