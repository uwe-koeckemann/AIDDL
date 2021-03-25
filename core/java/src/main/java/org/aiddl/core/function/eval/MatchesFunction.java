package org.aiddl.core.function.eval;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;

public class MatchesFunction implements PureFunction {

	@Override
	public Term apply(Term x) {
		return Term.bool( x.get(0).match( x.get(1) ) != null );
	}
}
