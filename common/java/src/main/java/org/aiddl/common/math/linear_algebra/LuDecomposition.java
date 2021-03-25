package org.aiddl.common.math.linear_algebra;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class LuDecomposition implements Function {

	@Override
	public Term apply(Term A_in) {
		if ( A_in.size() != A_in.get(0).size() ) {
			throw new IllegalArgumentException(String.format("Cannot perform LU decomposition on %dx%d matrix (format must be MxM)", A_in.size(), A_in.get(0).size()));
		}
		int n = A_in.size();
		
		List<List<Double>> A = new ArrayList<>();
		List<List<Double>> L = new ArrayList<>();
		List<List<Double>> U = new ArrayList<>();
		for ( int i = 0 ; i < n ; i++ ) {
			List<Double> A_row = new ArrayList<>();
			List<Double> L_row = new ArrayList<>();
			List<Double> U_row = new ArrayList<>();
			for ( int j = 0 ; j < n ; j++ ) {
				A_row.add(A_in.get(i).get(j).getDoubleValue());
				U_row.add(0.0);
				if ( i == j ) {
					L_row.add(1.0);
				} else {
					L_row.add(0.0);
				}
				
			}
			A.add(A_row);
			L.add(L_row);
			U.add(U_row);
		}
		
		for ( int k = 0 ; k < n ; k++ ) {
			double a_kk = A.get(k).get(k);
			U.get(k).set(k, a_kk);
			for ( int i = k + 1 ; i < n ; i++ ) {
				double a_ik = A.get(i).get(k);
				double a_ki = A.get(k).get(i);
												
				L.get(i).set(k, a_ik/a_kk);
				U.get(k).set(i, a_ki);
			}
			for ( int i = k + 1 ; i < n ; i++ ) {
				for ( int j = k + 1 ; j < n ; j++ ) {
					double a_ij = A.get(i).get(j);
					double l_ik = L.get(i).get(k);
					double u_kj = U.get(k).get(j);
					
					A.get(i).set(j, a_ij - l_ik*u_kj);
				}
			}
		}
		
		List<Term> U_r = new ArrayList<>(A.size());
		List<Term> L_r = new ArrayList<>(A.size());
		
		for ( int i = 0 ; i < A.size() ; i++ ) {
			List<Term> U_row = new ArrayList<>();
			List<Term> L_row = new ArrayList<>();
			
			for ( int j = 0 ; j < A.size() ; j ++ ) {
				U_row.add(Term.real(U.get(i).get(j)));
				L_row.add(Term.real(L.get(i).get(j)));
			}
			
			U_r.add(Term.tuple(U_row));
			L_r.add(Term.tuple(L_row));
		}
		 
		
		return Term.tuple(Term.tuple(L_r), Term.tuple(U_r));
	}

}
