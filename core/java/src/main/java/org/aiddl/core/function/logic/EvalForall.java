package org.aiddl.core.function.logic;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Uri;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.LazyFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

/**
 * Evaluate (forall x S c) which becomes true iff each x in collection term S satisfies c.
 * @author Uwe Koeckemann
 */
public class EvalForall implements LazyFunction, ConfigurableFunction {
	
	private static String ForallHelp = "Uses format: (forall (x S C)) where x is a term matched to all elements of collection term s.\nThe resulting terms must satisfy all constraints in set C.";
	
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public EvalForall( Evaluator main ) {
		this.eval = main;
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
	}
	
	@Override
	public Term apply(Term x) {
		if ( x.size() != 3 ) {
			throw new IllegalStateException(x + ": " + ForallHelp);
		}
		Term matchTerm = x.get(0);
		Term collTerm = eval.apply(x.get(1));
		
		if ( !(collTerm instanceof CollectionTerm) && !(collTerm instanceof TupleTerm) ) {
			throw new IllegalStateException(x.get(1) + " not a CollectionTerm or TupleTerm " + ForallHelp);
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
				return Term.bool(false);
			}
				
			Term conSub = constraints.substitute(s);

			if ( !this.eval.apply(conSub).getBooleanValue() ) {
				return Term.bool(false);
			}
		}
		return Term.bool(true);
	}

}
