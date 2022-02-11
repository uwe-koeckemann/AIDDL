package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class MatchesFunction implements Function {

	@Override
	public Term apply(Term x) {
		return Term.bool( x.get(0).match( x.get(1) ) != null );
	}
}
