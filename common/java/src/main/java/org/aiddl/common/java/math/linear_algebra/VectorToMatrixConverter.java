package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class VectorToMatrixConverter implements Function {

	@Override
	public Term apply(Term args) {
		LockableList M = new LockableList();
		for ( int i = 0 ; i < args.size() ; i++ ) {
			M.add(Term.tuple(args.get(i)));
		}
		return Term.tuple(M);
	}

}
