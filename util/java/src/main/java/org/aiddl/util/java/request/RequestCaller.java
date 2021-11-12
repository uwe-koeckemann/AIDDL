package org.aiddl.util.java.request;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

/**
 * Function that calls a request. This allows to make AIDDL requests available as packaged functions.
 * 
 * @author Uwe Koeckemann
 *
 */
public class RequestCaller implements Function {
	
	RequestHandler rHandler;
	Container C;
	
	public RequestCaller ( Container C, RequestHandler rHandler ) {
		this.rHandler = rHandler;
		this.C = C;
	}

	@Override
	public Term apply(Term args) {
		Term request = args.get(0);
		Term module = args.get(1);
		Term returnEntryName = args.get(2);
		
		this.C.addModule(module);

		this.rHandler.satisfyRequest(request, this.C, module);
		
		Term r = this.C.getEntry(module, returnEntryName).getValue();
		
		return r;
	}
}
