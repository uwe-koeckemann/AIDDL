package org.aiddl.common.java.planning.state_variable;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class GoalTest implements Function, InitializableFunction, InterfaceImplementation {
	SetTerm goal;
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.planning.state-variable.goal-test");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
		
	@Override
	public void initialize(Term args) {
		this.goal = args.asSet();
	}	
	
	@Override
	public Term apply(Term args) {
		for ( Term g : goal ) {
			Term s_a = args.asSet().get(g.getKey());
			if ( s_a == null || !s_a.equals(g.getValue()) ) {
				return Term.bool(false);
			}
		}
		return Term.bool(true);
	}
}
