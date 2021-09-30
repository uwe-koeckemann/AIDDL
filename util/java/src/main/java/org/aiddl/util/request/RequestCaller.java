package org.aiddl.util.request;

import org.aiddl.core.container.Container;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.util.request.RequestHandler;

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
