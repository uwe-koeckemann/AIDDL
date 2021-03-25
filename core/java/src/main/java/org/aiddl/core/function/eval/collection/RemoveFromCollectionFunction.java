package org.aiddl.core.function.eval.collection;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class RemoveFromCollectionFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return x.get(0).asCollection().remove(x.get(1));
	}
}
