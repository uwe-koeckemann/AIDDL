package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;

public class CollectionTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof CollectionTerm);
	}

}
