package org.aiddl.common.reasoning.temporal;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.reasoning.temporal.allen_constraints.Allen2STP;
import org.aiddl.common.reasoning.temporal.simple_temporal.STPSolver;
import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.Uri;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.unit_test.RunTests;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestTemporal extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
	
	public void testTemporal() {
		assertTrue( RunTests.testFile(aiddlTestStr + "/reasoning/temporal/test-cases.aiddl") );
	}
	
	public void testSTPs() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/reasoning/temporal/stp.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
		Logger.addPrintStream(System.out);
			
		STPSolver stpSolver = new STPSolver();
		
		for ( Entry e : db.getMatchingEntries(null, null, Term.tuple(Term.sym("stp-consistent"), Term.anonymousVar()))) {
			Term stp = e.getValue();
			
			
			assertTrue( eval.apply( fReg.getInputChecker(stpSolver.getInterfaceUri()), stp ).getBooleanValue() );
			Term answer = stpSolver.apply(stp);			
			assertTrue( eval.apply( fReg.getOutputChecker(stpSolver.getInterfaceUri()), answer ).getBooleanValue() );

			assertTrue( !answer.equals(CommonTerm.NIL));
		}
		
		for ( Entry e : db.getMatchingEntries(null, null, Term.tuple(Term.sym("stp-inconsistent"), Term.anonymousVar()))) {
			Term stp = e.getValue();
			assertTrue( eval.apply( fReg.getInputChecker(stpSolver.getInterfaceUri()), stp ).getBooleanValue() );
			Term answer = stpSolver.apply(stp);
			assertTrue( eval.apply( fReg.getOutputChecker(stpSolver.getInterfaceUri()), answer ).getBooleanValue() );

			assertTrue( answer.equals(CommonTerm.NIL));
		}
	}
	
	public void testAllenConstraints() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/reasoning/temporal/allen-interval-constraints.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
		Allen2STP aicConverter = new Allen2STP();
		STPSolver stpSolver = new STPSolver();
		
		for ( Entry e : db.getMatchingEntries(null, null, Term.tuple(Term.sym("consistent"), Term.anonymousVar()))) {
			Term AIC = e.getValue();
			assertTrue( eval.apply( fReg.getInputChecker(aicConverter.getInterfaceUri()), AIC ).getBooleanValue() );
			Term stp = aicConverter.apply(AIC);
			assertTrue( eval.apply( fReg.getOutputChecker(aicConverter.getInterfaceUri()), stp ).getBooleanValue() );

			assertTrue( eval.apply( fReg.getInputChecker(stpSolver.getInterfaceUri()), stp ).getBooleanValue() );
			Term answer = stpSolver.apply(stp);
			assertTrue( eval.apply( fReg.getOutputChecker(stpSolver.getInterfaceUri()), answer ).getBooleanValue() );

			assertTrue( !answer.equals(CommonTerm.NIL));
		}
		
		for ( Entry e : db.getMatchingEntries(null, null, Term.tuple(Term.sym("inconsistent"), Term.anonymousVar()))) {
			Term AIC = e.getValue();
			assertTrue( eval.apply( fReg.getInputChecker(aicConverter.getInterfaceUri()), AIC ).getBooleanValue() );
			Term stp = aicConverter.apply(AIC);
			assertTrue( eval.apply( fReg.getOutputChecker(aicConverter.getInterfaceUri()), stp ).getBooleanValue() );

			assertTrue( eval.apply( fReg.getInputChecker(stpSolver.getInterfaceUri()), stp ).getBooleanValue() );
			Term answer = stpSolver.apply(stp);
			assertTrue( eval.apply( fReg.getOutputChecker(stpSolver.getInterfaceUri()), answer ).getBooleanValue() );
			assertTrue( answer.equals(CommonTerm.NIL));
		}
	}
}
