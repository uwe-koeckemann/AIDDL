package org.aiddl.core.java.function.collection;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;

public class IsUniqueMapFunction implements Function {
	@Override
	public Term apply(Term x) {
		if ( x instanceof CollectionTerm )
			return Term.bool(x.asCollection().isUniqueMap());
		else if ( x instanceof TupleTerm ) 
			return Term.bool(x.asTuple().isUniqueMap());
		throw new IllegalArgumentException("IsUniqueMapFunction requires tuple or collection as first argument.");
	}
}
