package org.aiddl.common.learning.linear_regression;

import org.aiddl.common.math.linear_algebra.LupInverter;
import org.aiddl.common.math.linear_algebra.MatrixMultiplication;
import org.aiddl.common.math.linear_algebra.MatrixTranspose;
import org.aiddl.common.math.linear_algebra.MatrixVectorMultiplication;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class OrdinaryLeastSquaresRegression implements Function, InterfaceImplementation {

	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.learning.supervised.linear-regression");

	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public Term apply(Term args) {
		MatrixTranspose trans = new MatrixTranspose();
		LupInverter inv = new LupInverter();
		MatrixMultiplication mult = new MatrixMultiplication();
		MatrixVectorMultiplication mvMult = new MatrixVectorMultiplication();
		
		Term X = args.get(0);
		Term y = args.get(1);
		
		Term X_t = trans.apply(X);
		
		// (X^T X)^-1 X^T y
		Term w =  mvMult.apply(Term.tuple(
						mult.apply(Term.tuple(
								inv.apply(mult.apply(
										Term.tuple(X_t, 
												   X))), 
								X_t)), 
						y));		
		return w;
	}

}
