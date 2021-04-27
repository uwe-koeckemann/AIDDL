package org.aiddl.network;

import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

/**
 * Load gRPC function on request. Allows remote hosts to register their own functions on 
 * this server.
 * 
 * @author Uwe Koeckemann
 *
 */
public class GrpcFunctionLoader implements Function {
	private FunctionRegistry F;
	
	public GrpcFunctionLoader( FunctionRegistry F ) {
		this.F = F;
	}
	
	@Override
	public Term apply(Term t) {
		SymbolicTerm uri = t.get(Term.sym("uri")).asSym();
		String host = t.get(Term.sym("host")).getStringValue();
		int port = t.get(Term.sym("port")).getIntValue();
		
		GrpcFunction f = new GrpcFunction(host, port, uri);
		this.F.addFunction(uri, f);
		return Term.fref(uri, this.F);
	}

}
