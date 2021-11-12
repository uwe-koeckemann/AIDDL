package org.aiddl.common.java.reasoning.constraint;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class ValueOrdering implements Function, InterfaceImplementation {

	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.constraint.value-ordering");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public Term apply(Term args) {
		Term CSP = args.get(Term.sym("csp"));
		Term x = args.get(Term.sym("x"));
		
		CollectionTerm a = args.get(Term.sym("a")).asCollection();
		
		CollectionTerm X = CSP.get(0).asCollection();
		CollectionTerm D = CSP.get(1).asCollection();
		CollectionTerm C = CSP.get(2).asCollection();
		
		LockableList L = new LockableList();
		for ( Term e : D.get(x).asCollection() ) {
			L.add(Term.keyVal(x, e));
		}
		
		return Term.list(L);
	}
}
