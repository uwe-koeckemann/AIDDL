package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class ConcatFunction implements Function {

	@Override
	public Term apply(Term x) {
		LockableList S = new LockableList();
		for ( Term s : x.asList() ) {
			S.addAll(s.asList().getLockedList());
		}
		return Term.list(S);
	}

}
