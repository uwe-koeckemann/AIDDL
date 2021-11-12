package org.aiddl.common.java.learning.linear_regression;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.java.learning.LearningTerm;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.FunctionGenerator;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.IntegerTerm;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableList;

public class LinearRegression implements ConfigurableFunction, FunctionGenerator, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.learning.supervised.linear-regression.learner");
	
	IntegerTerm label_idx;
	FunctionReferenceTerm f_expand;
	Function f_e; 
	Term w = null; 
	FunctionRegistry fReg; 
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg;
		f_expand = settings.getOrDefault(Term.sym("expansion"), Term.fref(Uri.QUOTE, fReg)).asFunRef();

		f_e = fReg.getFunctionOrPanic(f_expand.getFunRefTerm());
	}
	
	@Override
	public Function generate() {
		LinearRegressionFunction lin_reg_fun = new LinearRegressionFunction();
		lin_reg_fun.configure(new HashMap<>(), fReg);
		lin_reg_fun.initialize(Term.tuple(this.w, this.f_expand, this.label_idx));
		return lin_reg_fun;
	}
	
	@Override
	public Term apply(Term args) {
		ListTerm attsTerm = (ListTerm) args.get(LearningTerm.Attributes);
		Term label = args.get(LearningTerm.Label);
		Term examples = args.get(LearningTerm.Data);
		
		int label_idx = -1;
		for ( int i = 0 ; i < attsTerm.size() ; i++ ) {
			if ( attsTerm.get(i).get(0).equals(label) ) {
				this.label_idx = Term.integer(i);
				label_idx = i;
				break;
			}
		}
				
		LockableList y_list = new LockableList();
		LockableList X_list = new LockableList();
		
		for ( int i = 0 ; i < examples.size() ; i++ ) {
			LockableList x_list = new LockableList();
			for ( int j = 0 ; j < examples.get(i).size() ; j++ ) {
				if ( j != label_idx ) {
					x_list.add(examples.get(i).get(j));
				}
			}
			Term x = Term.tuple(x_list);
			if ( f_expand != null ) {
				x = f_e.apply(x);
			}
			X_list.add(x);
			
			y_list.add(examples.get(i).get(label_idx));
		}
		
		OrdinaryLeastSquaresRegression ols = new OrdinaryLeastSquaresRegression();
		this.w = ols.apply(Term.tuple(Term.tuple(X_list), Term.tuple(y_list)));
		
		return w;
	}

	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
}
