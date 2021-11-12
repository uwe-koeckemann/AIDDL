package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;

public class VectorDotProduct implements Function {

	@Override
	public Term apply(Term args) {
		Term x = args.get(0);
		Term y = args.get(1);
		
		if ( x.size() != y.size() ) {
			throw new IllegalArgumentException(String.format("Dot product requires equal lengths vectors.\n\t%s\n\t%s", x.toString(), y.toString()));
		}
		
		NumericalTerm s = Term.integer(0);
		
		for ( int i = 0 ; i < x.size() ; i++ ) {
			s = s.add(x.get(i).asNum().mult(y.get(i).asNum()));
		}
		
		return s;
	}
	

}
