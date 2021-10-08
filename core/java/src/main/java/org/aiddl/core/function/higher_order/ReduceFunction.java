package org.aiddl.core.function.higher_order;

import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Uri;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.LazyFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.Term;

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
