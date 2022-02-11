package org.aiddl.core.java.function.higher_order;

import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.Term;

/**
 * Evaluate (map f {...}  c) to c with all xi substituted by vi.
 * 
 * @author Uwe Koeckemann
 */
public class ReduceFunction implements LazyFunction, ConfigurableFunction {
	
	Function eval;
//	FunctionRegistry fReg;
	Term InitValue = Term.sym("initial-value");
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public ReduceFunction( Evaluator main ) { //, FunctionRegistry fReg ) {
		this.eval = main;
//		this.fReg = fReg;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		FunctionReferenceTerm fun = this.eval.apply(x.get(0)).asFunRef();
		Function f = fun.getFunction();
		CollectionTerm C = this.eval.apply(x.get(1)).asCollection();

		Term current_value = x.get(InitValue);

		for ( Term e : C ) {
			if ( current_value == null ) {
				current_value = e;
			} else {
				current_value = f.apply(Term.tuple(current_value, e)); //this.eval.compute(f.substitute(s));	
			}
			
		}
		
		return current_value;
	}
}
