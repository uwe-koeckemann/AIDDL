package org.aiddl.common.java.math.statistics;

import org.aiddl.common.java.math.linear_algebra.EpsilonEquality;
import org.aiddl.common.java.math.linear_algebra.VectorToMatrixConverter;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestStatistics extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testMatrixVariance() {
		
		Term M = Parser.ParseTerm("((4.0 2.0 0.6)"
				+ "(4.2 2.1 0.59)"
				+ "(3.9 2.0 0.58)"
				+ "(4.3 2.1 0.62)"
				+ "(4.1 2.2 0.63)"
				+ ")");
		
		
		Term mean_real = Parser.ParseTerm(
				  "((4.10) (2.08) (0.604))"); 
		
		
		Term S_real = Parser.ParseTerm(
				  "((0.025 0.0075 0.00175)"
				+ "(0.0075 0.007 0.00135)"
				+ "(0.00175 0.00135 0.00043))"); 
		
		
		VectorMeanAndCovariance calc = new VectorMeanAndCovariance();
		VectorToMatrixConverter vConv = new VectorToMatrixConverter();
		
		
		Term stats = calc.apply(M);
		
		Term mean = vConv.apply(stats.get(0));
		Term S = stats.get(1);

		EpsilonEquality eq = new EpsilonEquality();
		assertTrue( eq.apply(Term.tuple(S, S_real, Term.real(0.0001))).getBooleanValue() );
		
		assertTrue( eq.apply(Term.tuple(mean, mean_real, Term.real(0.0001))).getBooleanValue() );
		

		
		
	}
}
