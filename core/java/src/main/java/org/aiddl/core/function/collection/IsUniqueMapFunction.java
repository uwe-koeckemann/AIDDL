package org.aiddl.core.function.collection;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

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
