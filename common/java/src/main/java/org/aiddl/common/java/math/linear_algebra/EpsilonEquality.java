package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class EpsilonEquality implements Function {

	@Override
	public Term apply(Term args) {
		Term A = args.get(0);
		Term B = args.get(1);
		NumericalTerm epsilon = args.get(2).asNum();
		
		if ( A.size() != B.size() || A.get(0).size() != B.get(0).size() ) {
			return Term.bool(false);
		}
		
		for ( int i = 0 ; i < A.size() ; i++ ) {
			for ( int j = 0 ; j < A.get(0).size() ; j++ ) {
				NumericalTerm diff = A.get(i).get(j).asNum().sub(B.get(i).get(j).asNum());
				if ( diff.isNegative() ) {
					diff = diff.mult(Term.integer(-1));
				}
				if ( diff.greaterThan(epsilon) ) {
					return Term.bool(false);
				}
			}
		}
		
		return Term.bool(true);
	}

}
