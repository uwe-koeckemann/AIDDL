package org.aiddl.core.java.function.logic;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;

/**
 * Evaluate (exists x S c) which becomes true iff collection term S contains an x that satisfies c. 
 * @author Uwe Koeeckemann
 */
public class EvalExists implements LazyFunction, ConfigurableFunction {
	
	private static String ExistsHelp = "Uses format: (exists (x S C)) where x is a term matched to all elements of collection term s.\nThe resulting terms must satisfy all constraints in set C.";
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public EvalExists( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply( Term x ) {
		if ( x.size() != 3 ) {
			throw new IllegalStateException(x + ": " + ExistsHelp);
		}
		
		Term matchTerm = x.get(0);
		Term collTerm = eval.apply(x.get(1));
		
		if ( !(collTerm instanceof CollectionTerm) && !(collTerm instanceof TupleTerm) ) {
			throw new IllegalStateException(x.get(1) + " not a CollectionTerm or TupleTerm " + ExistsHelp);
		} 
		
		CollectionTerm collection;
		if ( (collTerm instanceof CollectionTerm) ) {
			collection = collTerm.asCollection();
		} else {
			List<Term> list = new ArrayList<>();
			for ( int i = 0 ; i < collTerm.size() ; i++ ) {
				list.add(collTerm.get(i));
			}
			collection = Term.list(list);
		}
		Term constraints = x.get(2);

		for ( Term t : collection ) {
			Substitution s = matchTerm.match(t);

			if ( s == null ) {
				continue;
			}
			Term conSub = constraints.substitute(s);
	
			if ( this.eval.apply(conSub).getBooleanValue() ) {
				return Term.bool(true);
			}
		}
		return Term.bool(false);
	}
}
