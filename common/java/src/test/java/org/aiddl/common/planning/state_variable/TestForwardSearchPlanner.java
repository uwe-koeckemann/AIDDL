package org.aiddl.common.planning.state_variable;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.ForwardSearchPlanner;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestForwardSearchPlanner extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/";

	public void testSolvableProblem01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-01.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
				
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );


		
		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
	
	public void testSolvableProblem02() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-02.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
				
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );
		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
	
	public void testUnsolvableProblem03() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-03.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
				
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );

		assertTrue ( pi.equals(PlanningTerm.NIL) );
	}
	
	public void testSolvableProblem04() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/data-collection/problem-01.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
				
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );

		assertTrue(pi.size() == 9);
		
		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
	
	public void testSolvableProblem05() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/data-collection/problem-02.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
				
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );

		assertTrue(pi.size() == 7);
		
		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
	public void testSolvableProblem06() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/data-collection/problem-03.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
		
		Term cfg = Parser.ParseTerm("{heuristic:fast-forward}");
		
		planner.configure(cfg.asSet().getMap(), fReg);
				
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );

		assertTrue(pi.size() < 20);
		
		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
}
