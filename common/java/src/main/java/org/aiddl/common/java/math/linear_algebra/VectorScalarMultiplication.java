package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.NumericalTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class VectorScalarMultiplication implements Function {
	
	public Term apply( Term args ) {
		NumericalTerm lambda = args.get(0).asNum();
		Term x = args.get(1);
		
		LockableList r = new LockableList();
		for ( int i = 0 ; i < x.size() ; i++ ) {
			r.add(lambda.mult(x.get(i).asNum()));
		}
		return Term.tuple(r);
	}
}
