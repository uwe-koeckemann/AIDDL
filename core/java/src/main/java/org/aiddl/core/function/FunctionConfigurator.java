package org.aiddl.core.function;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class FunctionConfigurator implements Function {

	FunctionRegistry fReg;
	
	public FunctionConfigurator( FunctionRegistry fReg ) {
		this.fReg = fReg;
	}
		
	@Override
	public Term apply(Term args) {
		Term f = args.get(0);
		Term initArgs = args.get(1);
		Function func;
		SymbolicTerm uri;
		
		if ( f instanceof SymbolicTerm ) {
			func = this.fReg.getFunction(f.asSym());
			uri = f.asSym();
		} else if ( f instanceof FunctionReferenceTerm ) {
			func = f.asFunRef().getFunction();
			uri = f.asFunRef().getFunRefTerm();
		} else {
			throw new IllegalArgumentException("First argument of function inititializer must be symbolic (uri) or function reference term.");
		}
		
		if ( func instanceof InitializableFunction ) {
			((InitializableFunction)func).initialize(initArgs);
		} else {
			throw new IllegalArgumentException("Function not initializable.");
		}		
		
		return Term.fref(uri, this.fReg);
	}
}
