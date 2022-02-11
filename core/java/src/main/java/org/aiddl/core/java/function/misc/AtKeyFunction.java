package org.aiddl.core.java.function.misc;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class AtKeyFunction implements Function {

	Term Default = Term.sym("default");
	
	@Override
	public Term apply(Term x) {
		Term def = x.get(Default);
		if ( def == null ) {
			Term r = x.get(1).get(x.get(0));
			if ( r == null ) {
				throw new IllegalArgumentException("Key " + x.get(0) + " not found in " + x.get(1) + "\n\tyou may provide a default value by adding \"default:value\" to this functions arguments.");
			}
			return r;
		} 
		return x.get(1).getOrDefault(x.get(0), def);	
	}

}
