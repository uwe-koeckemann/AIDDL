package org.aiddl.core.function.eval;


import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Lambda;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class LambdaFunctionEvaluator implements Function {
	
	static int NextID = 0;
	
	FunctionRegistry fReg;
	Evaluator eval;
	
	public LambdaFunctionEvaluator( FunctionRegistry fReg ) {
		this.fReg = fReg;
		this.eval = (Evaluator)fReg.getFunction(DefaultFunctions.EVAL);
	}

	@Override
	public Term apply(Term args) {
		Term arg_term = args.get(0);
		Term fun_term = args.get(1);

		Function f = new Lambda(arg_term, fun_term, this.eval);
		SymbolicTerm uri = Term.sym("#lambda_" + NextID);
		NextID += 1;
		this.fReg.addFunction(uri, f);
		
		return Term.fref(uri, fReg);
	}
}
