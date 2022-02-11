package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class VectorAddition implements Function {

	@Override
	public Term apply(Term args) {
		Term v1 = args.get(0);
		Term v2 = args.get(1);
		
		if ( v1.size() != v2.size() ) {
			throw new IllegalArgumentException("Vectors must have equal size for addition.\n" + v1 + "\n" + v2 );
		}

		LockableList result = new LockableList();
		for ( int i = 0 ; i < v1.size() ; i++ ) {
			result.add(v1.get(i).asNum().add(v2.get(i).asNum()));
		}
		return Term.tuple(result);
	}
}
