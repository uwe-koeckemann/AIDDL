package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class CholeskyDecomposition implements Function {

	@Override 
	public Term apply(Term A) {
		LockableList L = new LockableList();
		
		for ( int i = 0 ; i < A.size() ; i++ ) {
			LockableList L_row = new LockableList();
			for ( int j = 0 ; j < A.get(0).size() ; j++ ) {
				if ( j > i ) {
					L_row.add(Term.integer(0));
				} else if ( i == j ) {
					double sum = 0.0;
					for ( int k = 0 ; k < j-1 ; k++ ) {
						
						double L_jk = L_row.get(k).getDoubleValue();
						sum += L_jk*L_jk;
					}
					
					Term L_jj = Term.real(Math.sqrt(A.get(j).get(j).getDoubleValue() - sum));

					L_row.add(L_jj);
				} else {
					double L_jj = L.get(j).get(j).getDoubleValue();
					double A_ij = A.get(i).get(j).getDoubleValue();
					
					double sum = 0.0;
					for ( int k = 0 ; k < j-1 ; k++ ) {
						double L_ik = L.get(i).get(k).getDoubleValue();
						double L_jk = L.get(j).get(k).getDoubleValue();
						
						sum += L_ik * L_jk;
					}
					
					Term L_ij = Term.real(1.0/L_jj * (A_ij - sum));
					L_row.add(L_ij);
				}
			}
			L.add(Term.tuple(L_row));
		}
		
		Term L_m = Term.tuple(L);
		MatrixTranspose trans = new MatrixTranspose();
		Term L_t = trans.apply(L_m);
		
		return Term.tuple(L_m, L_t);
	}

}
