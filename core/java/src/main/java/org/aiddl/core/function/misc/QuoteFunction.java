package org.aiddl.core.function.misc;

import org.aiddl.core.interfaces.LazyFunction;
import org.aiddl.core.representation.Term;

public class QuoteFunction implements LazyFunction {

	@Override
	public Term apply(Term x) {
		return x;
	}

}
