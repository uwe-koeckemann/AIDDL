package org.aiddl.common.learning.linear_regression;

import java.util.Map;

import org.aiddl.common.learning.LearningTerm;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.FunctionGenerator;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.IntegerTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class LinearRegression implements ConfigurableFunction, FunctionGenerator, InterfaceImplementation {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.learning.supervised.linear-regression.learner");
	
	IntegerTerm label_idx;
	FunctionReferenceTerm f_expand;
	Term w = null; 
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		f_expand = settings.getOrDefault(Term.sym("expansion"), Term.fref(DefaultFunctions.QUOTE, fReg)).asFunRef();
	}
	
	@Override
	public Function generate() {
		LinearRegressionFunction lin_reg_fun = new LinearRegressionFunction();
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
				x = f_expand.getFunction().apply(x);
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
