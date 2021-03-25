package org.aiddl.common.math.linear_algebra;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class MatrixScalarMultiplication implements Function {
	
	@Override
	public Term apply( Term args ) {
		Term A = args.get(0);
		NumericalTerm c = args.get(1).asNum();
		
		LockableList A_c = new LockableList();
		
		for ( int i = 0 ; i < A.size() ; i++ ) {
			LockableList row = new LockableList();
			for ( int j = 0 ; j < A.get(0).size() ; j++ ) {
				row.add(A.get(i).get(j).asNum().mult(c));
			}
			A_c.add(Term.tuple(row));
		}
		return Term.tuple(A_c);
	}	

}
