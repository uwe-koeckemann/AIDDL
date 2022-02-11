package org.aiddl.common.java.math.linear_algebra;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class MatrixMultiplication implements Function {
	@Override
	public Term apply( Term args ) {
		Term A = args.get(0);
		Term B = args.get(1);
		
		int m = A.size();
		int n_a = A.get(0).size();
		int n_b = B.size();
		int p = B.get(0).size();
		
		if ( n_a != n_b ) {
			throw new IllegalArgumentException(String.format("Cannot multiply %dx%d matrix with %dx%d matrix (format must be MxN and NxP).", m, n_a, n_b, p));
		}
		
		List<Term> C = new ArrayList<>();
		
		for ( int i = 0 ; i < m ; i++ ) {
			List<Term> C_row = new ArrayList<>(); 
			for ( int j = 0 ; j < p ; j++ ) {
				NumericalTerm c_ij = Term.real(0.0);
				for ( int k = 0 ; k < n_a ; k++ ) {
					c_ij = c_ij.add(Term.real(A.get(i).get(k).getDoubleValue() * B.get(k).get(j).getDoubleValue()));
				}
				C_row.add(c_ij);
			}
			C.add(Term.tuple(C_row));
		}
		
		return Term.tuple(C);
	}
}
