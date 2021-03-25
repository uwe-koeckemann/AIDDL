package org.aiddl.common.math.linear_algebra;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

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
