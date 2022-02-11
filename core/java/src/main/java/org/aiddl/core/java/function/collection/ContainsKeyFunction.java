package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;

public class ContainsKeyFunction implements Function {
	@Override
	public Term apply(Term x) {
		Term C = x.get(0);
		Term k = x.get(1);
		if ( C instanceof CollectionTerm )
			return Term.bool(C.asCollection().containsKey(k));
		else if ( C instanceof TupleTerm ) 
			return Term.bool(C.asTuple().containsKey(k));
		throw new IllegalArgumentException("Contains key requires tuple or collection as first argument.");
	}
}
