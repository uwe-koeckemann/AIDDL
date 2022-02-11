package org.aiddl.common.java.math.linear_algebra;

import java.util.Map;

import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class LinearEquationSolver implements ConfigurableFunction {
	
	String name = LinearEquationSolver.class.getSimpleName();
	private boolean verbose = false;
	

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
	}
	
	@Override
	public Term apply( Term args ) {
		
		Term A = args.get(Term.sym("A"));
		Term b = args.get(Term.sym("b"));
		
		if ( verbose ) Logger.msg(name, "Solving: " + A + "x = " + b);
		
		LupDecomposition lupDecomp = new LupDecomposition();
		
		Term LUP = lupDecomp.apply(A);
				
		Term L  = LUP.get(0);
		Term U  = LUP.get(1);
		Term pi = LUP.get(2);
		
		LupSolver lupSolver = new LupSolver();
		
		Term x = lupSolver.apply(Term.tuple(L, U, pi, b));
		
		return x;
	}
}
