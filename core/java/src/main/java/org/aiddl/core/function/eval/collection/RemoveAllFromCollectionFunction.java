package org.aiddl.core.function.eval.collection;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class RemoveAllFromCollectionFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return x.get(0).asCollection().removeAll(x.get(1).asCollection());
	}
}
