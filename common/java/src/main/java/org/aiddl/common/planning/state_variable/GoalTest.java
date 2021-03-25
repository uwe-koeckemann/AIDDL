package org.aiddl.common.planning.state_variable;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

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
