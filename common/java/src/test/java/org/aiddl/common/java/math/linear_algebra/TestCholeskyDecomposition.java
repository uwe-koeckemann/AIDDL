package org.aiddl.common.java.math.linear_algebra;

import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestCholeskyDecomposition extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testCholeskyDecomposition() {
		MatrixMultiplication mult = new MatrixMultiplication();
		
		Term A = Parser.ParseTerm("("
				+ "(0.25 0 0)"
				+ "(0 0.25 0)"
				+ "(0 0 0.25)"	
				+ ")");
		
		CholeskyDecomposition cd = new CholeskyDecomposition();
		
		Term LL = cd.apply(A);
		
		Term B = mult.apply(LL);
		
		assertTrue( A.size() == B.size() );
		assertTrue( A.get(0).size() == B.get(0).size() );
		
		for ( int i = 0 ; i < B.size() ; i++ ) {
			for ( int j = 0 ; j < B.get(0).size() ; j++ ) {
				double a_ij = A.get(i).get(j).getDoubleValue();
				double b_ij = B.get(i).get(j).getDoubleValue();
				
				assertTrue(Math.abs(a_ij - b_ij) < 0.0001);
			}
		}
	}
}
