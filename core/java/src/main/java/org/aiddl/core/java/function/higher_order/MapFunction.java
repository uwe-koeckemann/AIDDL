package org.aiddl.core.java.function.higher_order;

import java.util.Map;

import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.LazyFunction;
import org.aiddl.core.java.tools.LockableCollection;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.LockableSet;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.Term;

/**
 * Evaluate (map {... } c) to c with all xi substituted by vi.
 * 
 * @author Uwe Koeckemann
 */
public class MapFunction implements LazyFunction, ConfigurableFunction {
	
	Function eval;
	
	/**
	 * Create new evaluator.
	 * @param main evaluator for sub-expression
	 */
	public MapFunction( Evaluator main ) {
		this.eval = main;
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
		
		LockableCollection C_m;
		if ( C instanceof ListTerm ) {
			C_m = new LockableList();
		} else {
			C_m = new LockableSet();
		}
		
		for ( Term e : C ) {
			C_m.add(f.apply(e));
		}
		
		if ( C instanceof ListTerm ) {
			return Term.list(C_m);
		} else {
			return Term.set(C_m);
		}
	}
}
