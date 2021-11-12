package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.tools.LockableSet;
import org.aiddl.core.java.representation.Term;

public class UnionFunction implements Function {
	@Override
	public Term apply(Term x) {
		LockableSet S = new LockableSet();
		for ( Term s : x.asCollection() ) {
			S.addAll(s.asSet().getLockedSet());
		}
		return Term.set(S);
	}		
}
