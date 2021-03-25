package org.aiddl.common.reasoning.constraint;

import org.aiddl.common.CommonTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class VariableOrdering implements Function, InterfaceImplementation {

	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.constraint.variable-ordering");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public Term apply(Term args) {
		Term CSP = args.get(Term.sym("csp"));
		CollectionTerm a = args.get(Term.sym("a")).asCollection();
		CollectionTerm X = CSP.get(0).asCollection();
		CollectionTerm D = CSP.get(1).asCollection();
		CollectionTerm C = CSP.get(2).asCollection();
		
		for ( Term x : X ) {
			if ( !a.containsKey(x) ) {
				return x;
			}
		}
		return CommonTerm.NIL;
		
		
	}
}
