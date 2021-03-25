package org.aiddl.common.reasoning.temporal.allen_constraints;

import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class Timepoint2IntervalDomain implements PureFunction, InterfaceImplementation {
	
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
