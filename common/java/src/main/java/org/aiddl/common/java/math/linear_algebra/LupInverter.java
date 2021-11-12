package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class LupInverter implements Function {

	@Override
	public Term apply(Term A) {
	
		LupDecomposition lupDecomp = new LupDecomposition();
		
		Term LUP = lupDecomp.apply(A);
				
		Term L  = LUP.get(0);
		Term U  = LUP.get(1);
		Term pi = LUP.get(2);
		
		LupSolver lupSolver = new LupSolver();
		
		IdentityMatrix ident = new IdentityMatrix();
		Term I = ident.apply(Term.integer(A.size()));
		
		LockableList A_inv = new LockableList();
		
		for ( int i = 0 ; i < I.size() ; i++ ) {
			Term x_i = lupSolver.apply(Term.tuple(L, U, pi, I.get(i)));	
			A_inv.add(x_i);
		}
		
		return Term.tuple(A_inv);
	}

}
