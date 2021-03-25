package org.aiddl.common.math.linear_algebra;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class MatrixSubtraction implements Function {
	@Override
	public Term apply( Term args ) {
		Term A = args.get(0);
		Term B = args.get(1);
		
		int m_A = A.size();
		int n_A = A.get(0).size();
		int n_B = A.get(0).size();
		int m_B = B.get(0).size();
		
		if ( m_A != m_B || n_A != n_B ) {
			throw new IllegalArgumentException(String.format("Cannot subtract %dx%d matrix from %dx%d matrix (dimensions must match).", m_A, n_A, m_B, n_B));
		}
		
		List<Term> C = new ArrayList<>();
		
		for ( int i = 0 ; i < m_A ; i++ ) {
			List<Term> C_row = new ArrayList<>(); 
			for ( int j = 0 ; j < n_A ; j++ ) {
				NumericalTerm c_ij = Term.real(A.get(i).get(j).getDoubleValue() - B.get(i).get(j).getDoubleValue());
				C_row.add(c_ij);
			}
			C.add(Term.tuple(C_row));
		}
		
		return Term.tuple(C);
	}
}
