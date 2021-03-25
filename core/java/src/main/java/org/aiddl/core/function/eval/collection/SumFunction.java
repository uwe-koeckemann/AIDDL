package org.aiddl.core.function.eval.collection;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.Term;

public class SumFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		if ( x instanceof CollectionTerm ) {
			CollectionTerm Col = x.asCollection();
			NumericalTerm sum = Term.integer(0);
			for ( Term e : Col ) {
				sum = sum.add((NumericalTerm)e);
			}
			return sum;
		} else {
			NumericalTerm sum = Term.integer(0);
			for ( int i = 0 ; i < x.size() ; i++ ) {
				sum = sum.add((NumericalTerm)x.get(i));
			}
			return sum;
		}
	}
}
