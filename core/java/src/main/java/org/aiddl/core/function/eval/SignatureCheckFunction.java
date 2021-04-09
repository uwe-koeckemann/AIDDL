package org.aiddl.core.function.eval;

import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Evaluate (signature x [t1 t2 ...]) to true if x is a tuple
 * with values (x1 x2 ...) of type t1, t2, etc.
 * 
 * @author Uwe Koeckemann
 *
 */
public class SignatureCheckFunction implements Function, ConfigurableFunction {
		
	Function eval;
	Term TYPE = Term.sym("org.aiddl.eval.type");
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public SignatureCheckFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		Term targets = x.get(0);
		Term types = x.get(1);
		
		if ( !(targets instanceof TupleTerm) ) {
			return Term.bool(false);
		}
		
		int idx;
		for ( int i = 0 ; i < targets.size() ; i++ ) {
			idx = Math.min(i, types.size()-1);
			TupleTerm con = Term.tuple(TYPE, targets.get(i), types.get(idx));
			if ( !eval.apply(con).getBooleanValue() ) {
				return Term.bool(false);
			}
		}
		return Term.bool(true);
	}

}
