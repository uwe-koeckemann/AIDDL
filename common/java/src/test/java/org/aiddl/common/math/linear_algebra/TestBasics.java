package org.aiddl.common.math.linear_algebra;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Uri;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.unit_test.RunTests;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestBasics extends TestCase {
		
	Container db; 
	FunctionRegistry fReg;
	Evaluator eval;
	
	String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
	
	@Override
	public void setUp() throws Exception {
		db = new Container();
		fReg = DefaultFunctions.createDefaultRegistry(db);
		eval = (Evaluator)fReg.getFunction(Uri.EVAL);
	}

	@Override
	public void tearDown() throws Exception {
	}

	
	public void testLinearAlgebra() {
		Function matrixMult = new MatrixMultiplication();
		Function vectorMult = new MatrixVectorMultiplication();
		fReg.addFunction( Term.sym("org.aiddl.common.linear-algebra.matrix-mult"), matrixMult ); 
		fReg.addFunction( Term.sym("org.aiddl.common.linear-algebra.vector-mult"), vectorMult );
		assertTrue( RunTests.testFile(aiddlTestStr + "/math/linear-algebra/test-cases.aiddl", db, fReg ) );
	}
		
	public void testTranspose1() {
		MatrixTranspose trans = new MatrixTranspose();
		
		Term A = Parser.ParseTerm("("
								+ "(12 -51 4)"
								+ "(6 167 -68)"
								+ ")");
		
		Term A_t = Parser.ParseTerm("("
								+ "(12 6)"
								+ "(-51 167)"
								+ "(4 -68)"								
								+ ")");
		
		Term A_r = trans.apply(A);
		assertTrue(A_r.equals(A_t));
		
		A_r = trans.apply(A_r);
		assertTrue(A_r.equals(A));	
	}
	
	public void testTranspose2() {
		MatrixTranspose trans = new MatrixTranspose();
		
		Term A = Parser.ParseTerm("("
								+ "(1)"
								+ ")");
		
		Term A_r = trans.apply(A);
		assertTrue(A_r.equals(A));
		
		A_r = trans.apply(A_r);
		assertTrue(A_r.equals(A));	
	}
}
