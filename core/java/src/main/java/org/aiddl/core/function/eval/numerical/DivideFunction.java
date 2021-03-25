package org.aiddl.core.function.eval.numerical;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class DivideFunction implements PureFunction {
	@Override
	public Term apply(Term x) {
		if ( x.size() == 0 ) return Term.integer(1);
		else if ( x.size() == 1 ) return Term.integer(1).div(x.get(0).asNum());
		
		NumericalTerm prod = x.get(0).asNum();
		for ( int i = 1 ; i < x.size() ; i++ ) {
			prod = prod.div(x.get(i).asNum());
		}
		return prod;
	}
}
