package org.aiddl.common.java.reasoning.constraint;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class ConstraintChecker implements Function, InterfaceImplementation, ConfigurableFunction, InitializableFunction {
	
	private String loggerName = "ConstraintChecker";
	private int verbose = 1;

	private FunctionRegistry fReg;
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.constraint.constraint-tester");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	Function eval;
	
	SetTerm C;
	List<Function> C_fun = new LinkedList<Function>();
	List<TupleTerm> C_scope = new LinkedList<TupleTerm>();

	@Override
	public void initialize(Term args) {
		C = args.asSet();
		for ( Term c : C ) {
			C_scope.add(c.get(0).asTuple());		
			Function c_fun = c.get(1).asFunRef().getFunction(); //  fReg.getFunction(c.get(1));
			
			C_fun.add(c_fun);
		}
	}
		
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.loggerName = settings.getOrDefault(Term.sym("log-name"), Term.string(this.loggerName)).toString();
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
		this.eval = fReg.getFunction(settings.getOrDefault(Term.sym("eval"), Uri.EVAL));
		this.fReg = fReg;
	}

	@Override
	public Term apply(Term args) {
		ListTerm a = args.asList();
		
		Substitution a_sub = new Substitution(a);

		boolean c_sat = true;
		for ( int i = 0 ; i < C_scope.size() ; i++ ) {
			Term scope = C_scope.get(i).substitute(a_sub);
			
			if ( verbose >= 1 ) {
				Logger.msg(this.loggerName, "Scope: " + scope);
				Logger.msg(this.loggerName, "Constraint: " + C_fun.get(i));		
			}
			Term result = C_fun.get(i).apply(scope);
			if ( verbose >= 1 ) {
				Logger.msg(this.loggerName, "Result: " + result);					
			}

						
			if ( !result.getBooleanValue() ) {
				c_sat = false;
				break;
			}
		}
		
		return Term.bool(c_sat);
	}
}
