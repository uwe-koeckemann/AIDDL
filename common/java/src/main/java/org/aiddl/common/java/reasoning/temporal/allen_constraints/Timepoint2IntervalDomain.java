package org.aiddl.common.java.reasoning.temporal.allen_constraints;

import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class Timepoint2IntervalDomain implements Function, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.temporal.allen-interval.timepoint-2-interval");
		
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply(Term args) {
		LockableList L = new LockableList();
		for ( int i = 1 ; i < args.size() ; i+=2 ) {
			L.add(Term.keyVal(args.get(i).getKey().get(1), Term.tuple(args.get(i).getValue(), args.get(i+1).getValue())));
		}
		return Term.list(L);
	}


}
