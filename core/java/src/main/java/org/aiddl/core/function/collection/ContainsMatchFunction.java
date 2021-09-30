package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;

public class ContainsMatchFunction implements Function {

	@Override
	public Term apply(Term x) {
		Term target = x.get(1);
		for ( Term e : (CollectionTerm)x.get(0) ) {
			if ( target.match(e) != null ) {
				return Term.bool(true);
			}
		}
		return Term.bool(false);
	}
}
