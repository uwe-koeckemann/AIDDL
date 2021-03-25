package org.aiddl.core.function.eval.set;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;

public class UnionFunction implements PureFunction {
	@Override
	public Term apply(Term x) {
		LockableSet S = new LockableSet();
		for ( Term s : x.asCollection() ) {
			S.addAll(s.asSet().getLockedSet());
		}
		return Term.set(S);
	}		
}
