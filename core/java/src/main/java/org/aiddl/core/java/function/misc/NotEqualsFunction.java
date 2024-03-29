package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class NotEqualsFunction implements Function {

	@Override
	public Term apply(Term x) {
		return ( (x.get(0) instanceof NumericalTerm ) && (x.get(1) instanceof NumericalTerm) ) 
				? Term.bool(! ((NumericalTerm)x.get(0)).equalTo((NumericalTerm) x.get(1)))
				: Term.bool(! x.get(0).equals(x.get(1))); 
	}
}
