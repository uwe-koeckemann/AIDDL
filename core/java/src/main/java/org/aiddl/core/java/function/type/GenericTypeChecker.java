package org.aiddl.core.java.function.type;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class GenericTypeChecker implements Function {
	long nextFreeID = 0;
	SymbolicTerm uriBase;
	private Term genTypeDef;
	private Term genArgs;
	private Evaluator eval;
	private FunctionRegistry fReg;

	public GenericTypeChecker(SymbolicTerm uriBase, Term genArgs, Term typeDef, Evaluator eval, FunctionRegistry fReg ) {
		this.uriBase = uriBase;
		this.genArgs = genArgs;
		this.genTypeDef = typeDef;
		this.eval = eval;
		this.fReg = fReg;
	}

	@Override
	public Term apply(Term args) {
		Substitution s = this.genArgs.match(args);
		Term typeDef = genTypeDef.substitute(s);
		SymbolicTerm uri = uriBase.concat(Term.sym(String.valueOf(nextFreeID++)));
		Function typeCheckFun = new TypeChecker(typeDef, this.eval);
		fReg.addFunction(uri, typeCheckFun);
		//return Term.fref(uri, this.fReg);
		return uri;
	}
}
