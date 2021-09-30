package org.aiddl.core.function.string;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class StringConcat implements Function {

	@Override
	public Term apply(Term args) {
		StringBuilder sB = new StringBuilder();
		sB.append("\"");
		for ( int i = 0 ; i < args.size() ; i++ ) {
			sB.append(args.get(i).getStringValue());
		}
		sB.append("\"");
		return Term.string(sB.toString());
	}

}
