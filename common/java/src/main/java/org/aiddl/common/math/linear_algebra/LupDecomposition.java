package org.aiddl.common.math.linear_algebra;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class LupDecomposition implements Function {

	@Override
	public Term apply(Term A_in) {
		if ( A_in.size() != A_in.get(0).size() ) {
			throw new IllegalArgumentException(String.format("Cannot perform LUP decomposition on %dx%d matrix (format must be MxM)", A_in.size(), A_in.get(0).size()));
		}
		
		int n = A_in.size();
		List<Integer> pi = new ArrayList<>();
		List<List<Double>> A = new ArrayList<>();
		for ( int i = 0 ; i < n ; i++ ) {
			pi.add(i);
			List<Double> A_row = new ArrayList<>();
			for ( int j = 0 ; j < n ; j++ ) {
				A_row.add(A_in.get(i).get(j).getDoubleValue());
			}
			A.add(A_row);
		}
		
		for ( int k = 0 ; k < n ; k++ ) {
			double p = 0.0;
			Integer k_d = null;
			for ( int i = k ; i < n ; i++ ) {
				if ( Math.abs(A.get(i).get(k)) > p ) {
					p = Math.abs(A.get(i).get(k));
					k_d = i;
				}
			}
			if ( p == 0.0 ) {
				throw new IllegalStateException("Matrix is singular.");
			}
			{
				int tmp = pi.get(k);
				pi.set(k, pi.get(k_d));
				pi.set(k_d, tmp);
			}
			for ( int i = 0 ; i < n ; i++ ) {
				Double tmp = A.get(k).get(i);
				A.get(k).set(i, A.get(k_d).get(i));
				A.get(k_d).set(i, tmp);
			}
			for ( int i = k+1 ; i < n ; i++ ) {
				A.get(i).set(k, A.get(i).get(k)/A.get(k).get(k));
				for ( int j = k+1 ; j < n ; j++ ) {
					Double a_ij = A.get(i).get(j);
					Double a_kj = A.get(k).get(j);
					A.get(i).set(j, a_ij - A.get(i).get(k)*a_kj);					
				}
			}
		}
		
		List<Term> L = new ArrayList<>();
		List<Term> U = new ArrayList<>();
		for ( int i = 0 ; i < n ; i++ ) {
			List<Term> L_row = new ArrayList<>();
			List<Term> U_row = new ArrayList<>();
			for ( int j = 0 ; j < n ; j++ ) {
				if ( i > j ) {
					L_row.add(Term.real(A.get(i).get(j)));
					U_row.add(Term.real(0.0));
				} else if ( i == j ) {
					L_row.add(Term.real(1.0));
					U_row.add(Term.real(A.get(i).get(j)));
				} else {
					L_row.add(Term.real(0.0));
					U_row.add(Term.real(A.get(i).get(j)));
				}
			}
			L.add(Term.tuple(L_row));
			U.add(Term.tuple(U_row));
		}
				 
		List<Term> pi_r = new ArrayList<>();
		for ( int k : pi ) {
			pi_r.add(Term.integer(k));
		}
		
		return Term.tuple(Term.tuple(L), Term.tuple(U), Term.tuple(pi_r));
	}

}
