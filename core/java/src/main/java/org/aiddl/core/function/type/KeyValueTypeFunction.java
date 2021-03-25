package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.Term;

public class KeyValueTypeFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof KeyValueTerm);
	}

}
