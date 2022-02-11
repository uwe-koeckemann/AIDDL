package org.aiddl.common.java.math.linear_algebra;

import java.util.ArrayList;
import java.util.List;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class LupSolver implements Function {

	@Override
	public Term apply(Term args) {
		Term L = args.get(0);
		Term U = args.get(1);
		Term pi = args.get(2);
		Term b = args.get(3);
			
		Integer n = L.size();
		
		List<Term> x = new ArrayList<>();
		List<Term> y = new ArrayList<>();
		for ( int i = 0 ; i < n ; i++ ) {
			x.add(Term.real(0.0));
			y.add(Term.real(0.0));
		}
		
		for ( int i = 0 ; i < n ; i++ ) {
			double sum = 0.0;
			for ( int j = 0 ; j < i ; j++ ) {
				sum += L.get(i).get(j).getDoubleValue() * y.get(j).getDoubleValue();
			}
			double y_i = b.get(pi.get(i).getIntValue()).getDoubleValue() - sum;
			y.set(i, Term.real(y_i));
		}
		
		for ( int i = n-1; i >= 0 ; i-- ) {
			double sum = 0.0;
			for ( int j = i+1 ; j < n ; j++ ) {
				sum += U.get(i).get(j).getDoubleValue() * x.get(j).getDoubleValue();
			}
			double x_i = (y.get(i).getDoubleValue() - sum) / U.get(i).get(i).getDoubleValue();
			x.set(i, Term.real(x_i));
		}
		
		return Term.tuple(x);

	}
}
