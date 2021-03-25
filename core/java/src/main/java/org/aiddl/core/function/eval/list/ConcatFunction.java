package org.aiddl.core.function.eval.list;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class ConcatFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		LockableList S = new LockableList();
		for ( Term s : x.asList() ) {
			S.addAll(s.asList().getLockedList());
		}
		return Term.list(S);
	}

}
