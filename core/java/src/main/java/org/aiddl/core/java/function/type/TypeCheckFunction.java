package org.aiddl.core.java.function.type;

import java.util.Map;

import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.BooleanTerm;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

/**
 * Evaluator for (type x t) to true if x has type t, and false otherwise.
 * @author Uwe Koeckemann
 */
public class TypeCheckFunction implements Function, ConfigurableFunction {
		
	private static final SymbolicTerm SELF = Term.sym("#self");
	private static final SymbolicTerm SELF_ALT = Term.sym("#arg");
	
	Function eval;
	
	FunctionRegistry fReg;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		Term evalName = settings.getOrDefault(Term.sym("eval"), Uri.EVAL);
		this.eval = fReg.getFunction(evalName);
		this.fReg = fReg;
	}
	
	/**
	 * Create new evaluator.
	 * @param fReg registry to fetch evaluator for sub-expression
	 */
	public TypeCheckFunction( FunctionRegistry fReg ) {
		this.eval = fReg.getFunction(Uri.EVAL);
		this.fReg = fReg;
	}
		
	@Override
	public Term apply(Term x) {
		Term target = x.get(0);
//		if ( target.equals(SELF) ) {
//			return x;
//		}
		Term type = x.get(1);
		type = eval.apply(type);
		
//		Substitution selfSub = new Substitution();
//		selfSub.add(SELF, target);
//		type = mainEval.eval(type.substitute(selfSub));
		
		if ( type instanceof CollectionTerm ) {
			for ( Term t : type.asCollection() ) {
				if ( t.equals(BooleanTerm.FALSE ) ) {
					return Term.bool(false);
				}
				Term answer = checkType( t, target );
				if ( answer.equals(Term.bool(true)) ) {
					return answer;
				}
				
			}
			return Term.bool(false);
		}
		if ( type.equals(BooleanTerm.FALSE ) ) {
			return Term.bool(false);
		}
		return checkType( type, target );
		
//		return basicTypeChecks.getOrDefault(type, identity).eval(target);
	}
	
	private Term checkType( Term type, Term target ) {
		Function tCheck = null;
		if ( type instanceof SymbolicTerm ) {
			tCheck = fReg.getFunction(type);
			System.err.println("[W] Not a function reference: " + type + " for " + target + " (it is " + type.getClass().getSimpleName() +")");
		} else if ( type instanceof FunctionReferenceTerm ) { 
			tCheck = type.asFunRef().getFunction();
		}
		if ( tCheck != null ) {
			return tCheck.apply(target);
		}
		System.err.println("[W] Not a function reference: " + type + " for " + target + " (it is " + type.getClass().getSimpleName() +")");
		Substitution selfSub = new Substitution();
		selfSub.add(SELF, target);
		selfSub.add(SELF_ALT, target);
		return eval.apply(type.substitute(selfSub));
	}
}
