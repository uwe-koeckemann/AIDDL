package org.aiddl.core.function.eval.logic;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class NotFunction implements PureFunction {
	@Override
	public Term apply(Term x) {
		return Term.bool(!x.getBooleanValue());
	}		
}
