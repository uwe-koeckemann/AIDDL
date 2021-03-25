package org.aiddl.common.math.linear_algebra;

import org.aiddl.core.representation.Term;

public class MatrixTools {
	public static void printMatrix( Term A ) {
		for ( int i = 0 ; i < A.size() ; i++ ) {
			System.out.println(A.get(i));
		}
	}
}
