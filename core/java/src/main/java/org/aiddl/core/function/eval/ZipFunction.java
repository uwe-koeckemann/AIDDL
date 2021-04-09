package org.aiddl.core.function.eval;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Zips a list of lists (zip [L1 L2 ...]) to list of
 * tuples where the nth tuple contains the nth elements
 * of L1, L2, etc.    
 * 
 * @author Uwe Koeckemann
 */
public class ZipFunction implements Function, ConfigurableFunction {
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public ZipFunction( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		Term matchTuple = x.get(0);
		ListTerm zipTerm = (ListTerm) x.get(1); 
		Term subConstraint = x.get(2);
		
		zipTerm = (ListTerm) eval.apply(zipTerm);
		
		List<Term> zipList = new ArrayList<>();
		for ( Term t : zipTerm ) {
			if ( t instanceof ListTerm || t instanceof TupleTerm )
				zipList.add(t);
			else 
				throw new IllegalArgumentException("Cannot zip terms that are not tuples or lists. Problem was " + t + " in call " + x);
		}
		
		for ( int i = 0 ; i < zipList.size()-1 ; i++ ) {
			if ( zipList.get(i).size() != zipList.get(i+1).size() ) {
				return Term.bool(false);
			}
		}
		
		for ( int i = 0 ; i < zipList.get(0).size() ; i++ ) {
			Term[] zipArgs = new Term[zipList.size()];
			for ( int j = 0 ; j < zipList.size() ; j++ ) {
				zipArgs[j] = zipList.get(j).get(i);
			}
			TupleTerm zipTuple = Term.tuple(zipArgs);
			Substitution s = matchTuple.match(zipTuple);
			if ( ! eval.apply(subConstraint.substitute(s)).getBooleanValue() ) {
				return Term.bool(false);
			}
		}
		return Term.bool(true);
	}

}
