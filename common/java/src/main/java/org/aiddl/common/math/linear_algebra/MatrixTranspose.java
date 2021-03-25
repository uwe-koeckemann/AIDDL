package org.aiddl.common.math.linear_algebra;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class MatrixTranspose implements Function {

	@Override
	public Term apply(Term A) {
		LockableList A_trans = new LockableList();
		
		for ( int i = 0 ; i < A.get(0).size() ; i++ ) {
			LockableList row = new LockableList();
			for ( int j = 0 ; j < A.size() ; j++ ) {
				row.add(A.get(j).get(i));
			}
			A_trans.add(Term.tuple(row));
		}
		
		return Term.tuple(A_trans);
	}

}
