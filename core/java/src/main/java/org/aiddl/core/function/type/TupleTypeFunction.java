package org.aiddl.core.function.type;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

public class TupleTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof TupleTerm);
	}

}
