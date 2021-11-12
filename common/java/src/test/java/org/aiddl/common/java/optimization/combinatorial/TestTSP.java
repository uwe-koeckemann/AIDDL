package org.aiddl.common.java.optimization.combinatorial;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.common.java.math.graph.GraphTerm;
import org.aiddl.common.java.math.graph.UndirectedGraph2Dot;
import org.aiddl.common.java.optimization.combinatorial.tsp.TspGenerator;
import org.aiddl.common.java.optimization.combinatorial.tsp.TspMinRemainder;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.representation.Term;
import org.aiddl.util.java.request.RequestHandler;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.unit_test.RunTests;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestTSP extends TestCase {
	
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

	public void testTSP() {
		assertTrue( RunTests.testFile(aiddlTestStr + "/optimization/combinatorial/traveling-salesperson-problem/test.aiddl", db, fReg) );
	}
	
	public void testTspGenerator() {
		TspGenerator tspGen = new TspGenerator();
		
		int n = 5;
		
		Term tsp = tspGen.apply(
				Term.tuple(
						Term.integer(n), 
						Term.integer(1000), 
						Term.integer(1000)));
		
		
		Term N = tsp.get(GraphTerm.Nodes);
		Term E = tsp.get(GraphTerm.Edges);
		
//		System.out.println(Logger.prettyPrint(tsp, 0));
		
		assertTrue(N.size() == n);
		assertTrue(E.size() == (n*n-n)/2);
		
		
		UndirectedGraph2Dot u2dot = new UndirectedGraph2Dot();
		
		u2dot.apply(tsp);
	}
	
	public void testTspProblemBestRemainder() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n03-01.aiddl", db, fReg);
	
		Term problem = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		TspMinRemainder h = new TspMinRemainder();
		h.initialize(problem);
		
		assertTrue(h.apply(Term.list()).equals(Term.integer(490)));
	}
	
	public void testTspProblemN3_01() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n03-01.aiddl", db, fReg);
		Term solver_module = Term.sym("org.aiddl.common.optimization.combinatorial.tsp.tree-search");
		Parser.parseFile(Parser.getModuleFilename(solver_module), db, fReg);
		fReg.loadContainerDefintions(db);
		fReg.loadContainerInterfaces(db);
		Term run_module = Term.sym("run-module");		
		db.addModule(run_module);
		
		RequestHandler server = new RequestHandler( fReg );
		server.setEnforceTypeCheck(true);

		Term problem = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		Entry main = db.getEntry(solver_module, Term.tuple(Term.sym("main"), problem, run_module));
		server.satisfyRequest(main, db, run_module);		
		Term answer = db.getEntry(run_module, Term.sym("arg-best")).getValue();
		
//		System.out.println(answer);
		
		Function pathCost = fReg.getFunctionOrPanic(Parser.ParseTerm("org.aiddl.common.optimization.combinatorial.tsp.tree-search.path-cost"));
//		System.out.println(pathCost.compute(answer));
		
		assertTrue(pathCost.apply(answer).equals(Term.integer(770)));
		
		assertFalse( answer.equals(CommonTerm.NIL) );
	}
	
	public void testTspProblemN4_01() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n04-01.aiddl", db, fReg);
		Term solver_module = Term.sym("org.aiddl.common.optimization.combinatorial.tsp.tree-search");
		Parser.parseFile(Parser.getModuleFilename(solver_module), db, fReg);
		Term run_module = Term.sym("run-module");		
		db.addModule(run_module);
		
		RequestHandler server = new RequestHandler( fReg );
//		server.setVerbose(true);

		Term problem = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		Entry main = db.getEntry(solver_module, Term.tuple(Term.sym("main"), problem, run_module));
		server.satisfyRequest(main, db, run_module);		
		Term answer = db.getEntry(run_module, Term.sym("arg-best")).getValue();

		Function pathCost = fReg.getFunctionOrPanic(Parser.ParseTerm("org.aiddl.common.optimization.combinatorial.tsp.tree-search.path-cost"));

		assertTrue(pathCost.apply(answer).equals(Term.integer(998)));
	}
	
	public void testTspProblemN5_01() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n05-01.aiddl", db, fReg);
		Term solver_module = Term.sym("org.aiddl.common.optimization.combinatorial.tsp.tree-search");
		Parser.parseFile(Parser.getModuleFilename(solver_module), db, fReg);
		Term run_module = Term.sym("run-module");		
		db.addModule(run_module);
		
		RequestHandler server = new RequestHandler( fReg );
//		server.setVerbose(true);

		Term problem = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		Entry main = db.getEntry(solver_module, Term.tuple(Term.sym("main"), problem, run_module));
		server.satisfyRequest(main, db, run_module);		
		Term answer = db.getEntry(run_module, Term.sym("arg-best")).getValue();
		
		Function pathCost = fReg.getFunctionOrPanic(Parser.ParseTerm("org.aiddl.common.optimization.combinatorial.tsp.tree-search.path-cost"));

		assertTrue(pathCost.apply(answer).equals(Term.integer(998)));
	}
}
