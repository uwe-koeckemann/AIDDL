package org.aiddl.common.java.reasoning.constraint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.CollectionTerm;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.Substitution;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;

public class CspSolver implements Function, InterfaceImplementation, ConfigurableFunction {
	
	private String loggerName = "CspSolver";
	private int verbose = 1;
	
	Function eval;
	FunctionRegistry fReg;
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.reasoning.constraint.solver");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
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
		CollectionTerm X = args.get(0).asCollection();
		CollectionTerm D = args.get(1).asCollection();
		CollectionTerm C = args.get(2).asCollection();
		

		
		List<Term> X_list = new ArrayList<>();
		for ( Term x : X ) {
			X_list.add(x);
		}		
		Map<Term,List<Term>> D_map = new HashMap<>();
		for ( Term domain : D ) {
			D_map.put(domain.getKey(), domain.getValue().asList().getListCopy() );
		}
		

		
		Map<Term,List<Function>> C_map = new HashMap<>();
		Map<Term,List<Term>> C_scope = new HashMap<>();
		for ( Term c : C ) {
			TupleTerm scope = c.get(0).asTuple();
	
			FunctionReferenceTerm c_fun_term = eval.apply(c.get(1)).asFunRef();
			Function c_fun = c_fun_term.getFunction(); //  fReg.getFunctionOrPanic(c_fun_term);
			
//			C_fun.add(fReg.getFunction(c.get(1)));
			
			for ( int i = 0 ; i < scope.size() ; i++ ) {
				 C_map.putIfAbsent(scope.get(i), new ArrayList<>());
				 C_map.get(scope.get(i)).add(c_fun);
				 C_scope.putIfAbsent(scope.get(i), new ArrayList<>());
				 C_scope.get(scope.get(i)).add(scope);			
			}
		}
		Substitution a = new Substitution();
		
		Substitution solution = solve(X_list, D_map, C_map, C_scope, a);
		
		if ( solution == null ) {
			return CommonTerm.NIL;
		} else {
			return solution.getTerm();
		}
	}
	
	private static List<Function> EmptyList = new LinkedList<>();
	private static List<Term> EmptyTermList = new LinkedList<>();
	
	private Substitution solve( List<Term> X, Map<Term,List<Term>> D, Map<Term,List<Function>> C_map, Map<Term,List<Term>> C_scope, Substitution a ) {
		if ( X.isEmpty() ) {
			if ( verbose >= 1 ) {
				Logger.msg(this.loggerName, "solution="+a);
			}
			return a;
		}
		Term x = X.get(0);
		X.remove(0);
		
		if ( verbose >= 1 ) {
			Logger.msg(this.loggerName, "a="+a);
			Logger.msg(this.loggerName, "x="+x);
			Logger.incDepth();
		}
		
		for ( Term v : D.get(x) ) {
			if ( verbose >= 1 ) Logger.msg(this.loggerName, "v="+v);
			Substitution a_next = a.copy();
			a_next.add(x, v);
			
			if ( verbose >= 1 ) Logger.incDepth();
			
			boolean c_sat = true;
			
			List<Function> C_x = C_map.getOrDefault(x, EmptyList);
			List<Term> C_scopes_x = C_scope.getOrDefault(x, EmptyTermList);
			for ( int i = 0 ; i < C_x.size() ; i++ ) {
				Function c_fun = C_x.get(i);
				Term args = C_scopes_x.get(i).substitute(a_next);
				
				Term result = c_fun.apply(args);
				
				if ( verbose >= 1 ) {
					Logger.msg(this.loggerName, "Constraint: " + c_fun);
					Logger.msg(this.loggerName, "Result: " + result);					
				}
				
				
				if ( !result.getBooleanValue() ) {
					c_sat = false;
					break;
				}
			}
			if ( verbose >= 1 ) Logger.decDepth();
			
			if ( c_sat ) {
				Substitution solution = solve(X, D, C_map, C_scope, a_next);
				if ( solution != null ) {
					if ( verbose >= 1 ) Logger.decDepth();
					return solution;
				}
			}
		}
		X.add(x);
		if ( verbose >= 1 ) Logger.decDepth();
		return null;
	}
}
