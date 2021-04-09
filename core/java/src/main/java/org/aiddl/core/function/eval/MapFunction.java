package org.aiddl.core.function.eval;

import java.util.Map;

import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableCollection;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableSet;

/**
 * Evaluate (map {... } c) to c with all xi substituted by vi.
 * 
 * @author Uwe Koeckemann
 */
public class MapFunction implements Function, ConfigurableFunction {
	
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
		Term evalName = settings.getOrDefault(Term.sym("eval"), DefaultFunctions.EVAL);
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
