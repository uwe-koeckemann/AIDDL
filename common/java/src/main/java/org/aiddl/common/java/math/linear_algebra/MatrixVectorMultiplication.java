package org.aiddl.common.java.math.linear_algebra;

import java.util.ArrayList;
import java.util.List;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class MatrixVectorMultiplication implements Function {
	
	@Override
	public Term apply( Term args ) {
		Term A = args.get(0);
		Term x = args.get(1);
		
		if ( A.get(0).size() != x.size() ) {
			throw new IllegalArgumentException(String.format("Cannot multiply %dx%d matrix with %d vector (format must be MxN for length N vector.", A.size(), A.get(0).size(), x.size()));
		}
		
		List<Term> b = new ArrayList<Term>();	
		for ( int i = 0 ; i < A.size() ; i++ ) {
			NumericalTerm b_i = Term.real(0.0);
			Term a_i = A.get(i);
			for ( int j = 0 ; j < a_i.size() ; j++ ) {
				NumericalTerm x_j = (NumericalTerm) x.get(j);
				NumericalTerm a_i_j = (NumericalTerm) a_i.get(j);
				b_i = b_i.add( a_i_j.mult(x_j) );
			}
			b.add(b_i);
		}
		return Term.tuple(b);
	}

//	public static Term vectorMult( Term A, Term x ) {
//		if ( A.get(0).size() != x.size() ) {
//			throw new IllegalArgumentException(String.format("Cannot multiply %dx%d matrix with %d vector (format must be MxN for length N vector.", A.size(), A.get(0).size(), x.size()));
//		}
//		List<Term> b = new ArrayList<Term>();	
//		for ( int i = 0 ; i < A.size() ; i++ ) {
//			NumericalTerm b_i = Term.real(0.0);
//			Term a_i = A.get(i);
//			for ( int j = 0 ; j < a_i.size() ; j++ ) {
//				NumericalTerm x_j = (NumericalTerm) x.get(j);
//				NumericalTerm a_i_j = (NumericalTerm) a_i.get(j);
//				b_i = b_i.add( a_i_j.mult(x_j) );
//			}
//			b.add(b_i);
//		}
//		return Term.tuple(b);
//	}
	

}
