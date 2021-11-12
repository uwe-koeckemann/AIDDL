package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.Term;

public class NormL implements Function {
	
	int n = 2;
	
	public NormL( int n ) {
		this.n = n;
	}
	
	public Term apply( Term x ) {
		double s = 0.0;
		double n_r = (double)n;
		for ( int i = 0 ; i < x.size() ; i++ ) {
			double v = x.get(i).getDoubleValue();
			s += Math.pow(Math.abs(v), n_r);
		}
		return Term.real(Math.pow(s, 1.0/n_r));
	}
}
