package org.aiddl.common.planning.state_variable;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.util.request.RequestHandler;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestWhiteBoxPlanner extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/";

	public void testWhiteboxPlannerFunctions() {
		Term a = Parser.ParseTerm("(name:a1 preconditions:{x:(A:a) y:(A:b)} effects:{x:(A:c)})");
				
		Term s1 = Parser.ParseTerm("{x:(A:a) y:(A:b)}");
		Term s2 = Parser.ParseTerm("{x:(A:b) y:(A:b)}");
		
		ApplicableFunction app = new ApplicableFunction();
		Term b1 = app.apply(Term.tuple(a, s1));
		
		assertTrue( b1.getBooleanValue() );
		
		Term b2 = app.apply(Term.tuple(a, s2));
		
		assertTrue( !b2.getBooleanValue() );
		
		GoalTest goalTest = new GoalTest();
		
		Term g1 = Parser.ParseTerm("{x:(A:a) y:(A:b)}");
		goalTest.initialize(g1);
		assertTrue( goalTest.apply(s1).getBooleanValue()) ;
				
		Term g2 = Parser.ParseTerm("{x:(A:a)}");
		goalTest.initialize(g2);
		assertTrue( goalTest.apply(s1).getBooleanValue()) ;
		
		Term g3 = Parser.ParseTerm("{y:(A:b)}");
		goalTest.initialize(g3);
		assertTrue( goalTest.apply(s1).getBooleanValue()) ;
		
		Term g4 = Parser.ParseTerm("{}");
		goalTest.initialize(g4);
		assertTrue( goalTest.apply(s1).getBooleanValue()) ;
		
		Term g5 = Parser.ParseTerm("{x:(A:b)}");
		goalTest.initialize(g5);
		assertTrue( !goalTest.apply(s1).getBooleanValue()) ;
		
		StateTransitionFunction stf = new StateTransitionFunction();
		
		Term s3 = stf.apply(Term.tuple(s1, a));
		assertTrue( s3.equals(Parser.ParseTerm("{x:(A:c) y:(A:b)}")));
		
		Expand stateExpansion = new Expand();
		stateExpansion.initialize(Term.set(a));
		
		Term exp1 = stateExpansion.apply(s1);
		assertTrue(exp1.size() == 1);
		assertTrue(exp1.asList().contains(Term.tuple(Term.sym("a1"), s3)));
		
		Term exp2 = stateExpansion.apply(s2);
		assertTrue(exp2.size() == 0);
	}
	
	public void testWhiteboxPlannerSolvable01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module = Parser.parseFile("../test/planning/state-variable/elevator/problem-01.aiddl", db, fReg);
		Term planner_module = Term.sym("org.aiddl.common.planning.state-variable.solver.forward-search"); 
		
		Parser.parseFile(Parser.getModuleFilename(planner_module), db, fReg);
		Term exec_module = Term.sym("org.aiddl.examples.run-module");		
		db.addModule(exec_module);
		
		RequestHandler server = new RequestHandler( fReg );
		
		Term Pi = db.getEntry(data_module, Term.sym("problem")).getValue();
		Term s0 = Pi.get(PlanningTerm.InitialState);
		Term g = Pi.get(PlanningTerm.Goal);
		Term O = Pi.get(PlanningTerm.Operators);
		
//		System.out.println(Pi);
	
		Entry main = db.getEntry(planner_module, Term.tuple( Term.sym("main"), Term.tuple(s0, g, O), exec_module ));
		server.satisfyRequest(main, db, exec_module);
		Term pi = db.getEntry(exec_module, Term.sym("pi")).getValue();
		assertTrue( !pi.equals(CommonTerm.NIL) );
	}
	public void testWhiteboxPlannerSolvable02() {
//		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module = Parser.parseFile("../test/planning/state-variable/elevator/problem-02.aiddl", db, fReg);
		Term planner_module = Term.sym("org.aiddl.common.planning.state-variable.solver.forward-search"); 
		
		Parser.parseFile(Parser.getModuleFilename(planner_module), db, fReg);
		Term exec_module = Term.sym("org.aiddl.examples.run-module");		
		db.addModule(exec_module);
	
		RequestHandler server = new RequestHandler( fReg );
		
		Term Pi = db.getEntry(data_module, Term.sym("problem")).getValue();
		Term s0 = Pi.get(PlanningTerm.InitialState);
		Term g = Pi.get(PlanningTerm.Goal);
		Term O = Pi.get(PlanningTerm.Operators);
		
//		System.out.println(Pi);
	
		Entry main = db.getEntry(planner_module, Term.tuple( Term.sym("main"), Term.tuple(s0, g, O), exec_module ));
		server.satisfyRequest(main, db, exec_module);
		Term pi = db.getEntry(exec_module, Term.sym("pi")).getValue();
		assertTrue( !pi.equals(CommonTerm.NIL) );
	}
	public void testWhiteboxPlannerUnsolvable01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module = Parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl", db, fReg);
		Term planner_module = Term.sym("org.aiddl.common.planning.state-variable.solver.forward-search"); 
		
		Parser.parseFile(Parser.getModuleFilename(planner_module), db, fReg);
		Term exec_module = Term.sym("org.aiddl.examples.run-module");		
		db.addModule(exec_module);
	
		RequestHandler server = new RequestHandler( fReg );
		
		Term Pi = db.getEntry(data_module, Term.sym("problem")).getValue();
		Term s0 = Pi.get(PlanningTerm.InitialState);
		Term g = Pi.get(PlanningTerm.Goal);
		Term O = Pi.get(PlanningTerm.Operators);
		
		Entry main = db.getEntry(planner_module, Term.tuple( Term.sym("main"), Term.tuple(s0, g, O), exec_module ));
		server.satisfyRequest(main, db, exec_module);
		Term pi = db.getEntry(exec_module, Term.sym("pi")).getValue();
		assertTrue( pi.equals(CommonTerm.NIL) );
	}
}
