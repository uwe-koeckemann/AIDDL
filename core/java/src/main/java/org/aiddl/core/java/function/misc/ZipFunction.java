package org.aiddl.core.java.function.misc;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;

/**
 * Zips a list of lists (zip [L1 L2 ...]) to list of
 * tuples where the nth tuple contains the nth elements
 * of L1, L2, etc.    
 * 
 * @author Uwe Koeckemann
 */
public class ZipFunction implements LazyFunction, ConfigurableFunction {
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
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		ListTerm zipTerm = (ListTerm) x; 
		
		zipTerm = (ListTerm) eval.apply(zipTerm);
		
		List<Term> zipList = new ArrayList<>();
		for ( Term t : zipTerm ) {
			if ( t instanceof ListTerm || t instanceof TupleTerm )
				zipList.add(t);
			else 
				throw new IllegalArgumentException("Cannot zip terms that are not tuples or lists. Problem was " + t + " in call " + x);
		}
		LockableList zippedList = new LockableList();
		int idx = 0; 
		boolean done = false;
		while ( !done ) { 
			LockableList zipLine = new LockableList();
			for ( int i = 0 ; i < zipList.size() ; i++ ) {
				if ( idx >= zipList.get(i).size() ) {
					done = true;
					break;
				}
				zipLine.add(zipList.get(i).get(idx));
			}
			if ( !done ) {
				zippedList.add(Term.tuple(zipLine));
			}
			idx++;
		}
		return Term.list(zippedList);
	}
}
