package org.aiddl.common.java.planning.state_variable;

import org.aiddl.common.java.planning.PlanningFunctionLoader;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
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
		FunctionRegistry fReg = db.getFunctionRegistry();
		PlanningFunctionLoader.register(fReg, db);
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-01.aiddl", db);
        
		Evaluator eval = db.evaluator();
		
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
		FunctionRegistry fReg = db.getFunctionRegistry();
		PlanningFunctionLoader.register(fReg, db);

		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-02.aiddl", db);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
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
		FunctionRegistry fReg = db.getFunctionRegistry();
		PlanningFunctionLoader.register(fReg, db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-03.aiddl", db);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
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
		FunctionRegistry fReg = db.getFunctionRegistry();
		PlanningFunctionLoader.register(fReg, db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/data-collection/problem-01.aiddl", db);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
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
		FunctionRegistry fReg = db.getFunctionRegistry();
		PlanningFunctionLoader.register(fReg, db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/data-collection/problem-02.aiddl", db);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
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
		FunctionRegistry fReg = db.getFunctionRegistry();
		PlanningFunctionLoader.register(fReg, db);

		Parser.parseFile(aiddlTestStr + "/planning/state-variable/data-collection/problem-03.aiddl", db);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
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
