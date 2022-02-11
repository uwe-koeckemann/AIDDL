package org.aiddl.core.java.function;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

/**
 * Wrap a container into a function to allow access via apply interface
 * 
 * @author Uwe Koeckemann
 *
 */
public class ContainerFunction implements Function {
	
	Container C;
	Function F;
	
	public ContainerFunction( Container C, Function F ) {
		this.C = C;
		this.F = F;
	}

	@Override
	public Term apply(Term t) {
		Term op = t.get(0);
		
		if ( op.equals(Term.sym("get-entry")) ) {
			Term name = t.get(1);
			Term module = t.getOrDefault(Term.sym("module"), this.C.getWorkingModule());
			this.C.getEntry(module, name);
		}
		
		return null;
	}

}
