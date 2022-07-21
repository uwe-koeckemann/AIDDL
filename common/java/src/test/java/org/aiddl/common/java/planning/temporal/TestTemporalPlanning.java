package org.aiddl.common.java.planning.temporal;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestTemporalPlanning extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
	
	public void testTemporalPlanning() {

		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = db.getFunctionRegistry();
		
		Term data_module = Parser.parseFile(aiddlTestStr + "/planning/temporal/elevator/problem-01.aiddl", db);
//		Parser.parseFile(System.getenv("AIDDL_TEST") + "/planning/state-variable/dock-worker-robot/problem-02.aiddl", db);		
		Term planner_module = Term.sym("org.aiddl.common.planning.temporal.planner"); 
		
		Parser.parseFile(Parser.getModuleFilename(planner_module), db);
		Term exec_module = Term.sym("org.aiddl.examples.run-module");		
		db.addModule(exec_module);

		Evaluator eval = db.evaluator();
		RequestHandler server = new RequestHandler( fReg );
		
		Term problem = eval.apply(db.getEntry(data_module, Term.sym("problem")).getValue());
		
//		Term data = problem.get(Term.sym("problem"));
		Term O = problem.get(Term.sym("operators")).asSet();
		SetTerm s0 = problem.get(Term.sym("constraints")).get(Term.sym("statement")).asSet();
		SetTerm goal = problem.get(Term.sym("constraints")).get(Term.sym("open-goal")).asSet();
		
//		System.out.println("O: " + O);
//		System.out.println("s0: " + s0);
//		System.out.println("g: " + goal);
		
//		Term svaProblem = Term.tuple(
//				Term.keyVal(PlanningTerm.Operators, problem.get(PlanningTerm.Operators)),
//				Term.keyVal(PlanningTerm.InitialState, s0),
//				Term.keyVal(PlanningTerm.Goal, goal));
				
		Entry main = db.getEntry(planner_module, Term.tuple( Term.sym("main"), Term.tuple(s0, goal, O), exec_module ));
		server.satisfyRequest(main, db, exec_module);
	
		Term pi = db.getEntry(exec_module, Term.sym("pi")).getValue();
//		System.out.println("PI: " + pi);
//		for ( Term a : pi.asCollection() ) {
//			System.out.println(a);
//		}
//		
		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
}
