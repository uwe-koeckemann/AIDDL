package org.aiddl.core.function.eval.math;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.IntegerTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class ExptFunction implements Function {

	@Override
	public Term apply(Term args) {
		NumericalTerm x = args.get(0).asNum();
		NumericalTerm n = args.get(1).asNum();
		
		if ( n.isZero() ) {
			return Term.integer(1);
		}
		NumericalTerm r;
		
		if ( !(n instanceof IntegerTerm) ) {
			double exp = n.getDoubleValue();
			r = Term.real(Math.pow(x.getDoubleValue(), exp));
		} else {
			r = x;
			for ( int i = 1 ; i < n.getIntValue() ; i++ ) {
				r = r.mult(x);
			}
		}
		
		return r;
	}
}
