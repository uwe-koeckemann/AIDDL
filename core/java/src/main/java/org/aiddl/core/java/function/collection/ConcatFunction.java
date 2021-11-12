package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.representation.Term;

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
