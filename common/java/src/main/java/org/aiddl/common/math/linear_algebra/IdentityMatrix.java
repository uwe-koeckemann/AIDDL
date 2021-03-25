package org.aiddl.common.math.linear_algebra;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class IdentityMatrix implements Function {

	@Override
	public Term apply(Term args) {
		LockableList I = new LockableList();
		
		for ( int i = 0 ; i < args.getIntValue() ; i++ ) {
			LockableList row = new LockableList();
			for ( int j = 0 ; j < args.getIntValue() ; j++ ) {
				row.add(Term.integer(i == j ? 1 : 0));
			}
			I.add(Term.tuple(row));
		}
		
		return Term.tuple(I);
	}

}
