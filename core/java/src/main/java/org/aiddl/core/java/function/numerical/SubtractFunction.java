package org.aiddl.core.java.function.numerical;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class SubtractFunction implements Function {

	@Override
	public Term apply(Term x) {
		
		if ( x.size() == 0 ) return Term.integer(0);
		else if ( x.size() == 1 ) return Term.integer(0).sub(x.get(0).asNum());
		
		NumericalTerm sum = x.get(0).asNum();
		for ( int i = 1 ; i < x.size() ; i++ ) {
			sum = sum.sub(x.get(i).asNum());
		}
		return sum;
	}
}
