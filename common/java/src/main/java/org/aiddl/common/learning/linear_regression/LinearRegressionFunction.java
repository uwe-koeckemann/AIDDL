package org.aiddl.common.learning.linear_regression;

import java.util.Map;

import org.aiddl.common.math.linear_algebra.VectorDotProduct;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableList;

public class LinearRegressionFunction implements InterfaceImplementation, InitializableFunction, ConfigurableFunction {
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.learning.supervised.regression-function");
	
	private int label_idx;
	private Term w;
	private FunctionReferenceTerm expansion;
	private Function f_e;
	VectorDotProduct prod = new VectorDotProduct();
	FunctionRegistry fReg;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg;
	}
		
	@Override
	public void initialize(Term args) {
		this.w = args.get(0);
		this.expansion = args.get(1).asFunRef();
		this.label_idx = args.get(2).getIntValue();

		f_e = fReg.getFunctionOrPanic(expansion.getFunRefTerm());
	}
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public Term apply(Term x) {
		LockableList x_list = new LockableList();
		for ( int i = 0 ; i < x.size() ; i++ ) {
			if ( i != label_idx ) {
				x_list.add(x.get(i));
			}
		}
		x = f_e.apply(Term.tuple(x_list));
		return prod.apply(Term.tuple(x, w));
	}

}
