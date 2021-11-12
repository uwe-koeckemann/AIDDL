package org.aiddl.core.java.function.type;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.Term;

public class FunctionReferenceTypeFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool(x instanceof FunctionReferenceTerm);
	}

}
