package org.aiddl.common.java.math.statistics;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class MeanAndVariance implements Function {

	@Override
	public Term apply(Term args) {
		NumericalTerm sum = Term.integer(0);
		for ( Term x : args.asCollection() ) {
			sum.add(x.asNum());
		}
		NumericalTerm mean = sum.div(Term.integer(args.size()));
		
		sum = Term.integer(0);
		for ( Term x : args.asCollection() ) {
			NumericalTerm diff = mean.sub(x.asNum());
			sum.add(diff.mult(diff));
		}
		NumericalTerm variance = sum.div(Term.integer(args.size()-1));
		return Term.tuple(mean, variance);
	}
}
