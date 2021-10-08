package org.aiddl.common.reasoning.constraint;

import java.util.Map;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.reasoning.constraint.CspSolver;
import org.aiddl.core.container.Container;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.Uri;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestCSP extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/";
	
	public void testMapColoring() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/reasoning/constraint/map-coloring.aiddl", db, fReg);
		
		CspSolver cspSolver = new CspSolver();
		Term conf_term = Parser.ParseTerm("[eval:org.aiddl.eval]");
		Map<Term,Term> config = conf_term.asList().getMap();
		cspSolver.configure(config , fReg );
	
		((Evaluator)fReg.getFunction(Uri.EVAL)).setContainer(db);
			
		Term csp = db.getEntry(Term.sym("test-1")).getValue().resolve(db);
				
//		assertTrue ( cspSolver.declaration().checkInput(arg, db) );
		Term answer = cspSolver.apply(csp);
		
//		System.out.println(answer);
//		assertTrue ( cspSolver.declaration().checkOutput(answer, db) );
				
		assertTrue ( answer.size() == 7 );
	}
	
//	public void testMapColoringTreeSearchSolver() {
//		Container db = new Container();
//		Parser.parseFile("reasoning/constraint/csp-solver.aiddl", db);
//		Parser.parseFile(aiddlTestStr + "/reasoning/constraint/map-coloring.aiddl", db);
//		
//		Evaluator eval = new Evaluator(db);
//	
//		CspSolver cspSolver = new CspSolver();
//		cspSolver.setEvaluator(eval);
//		
//		Term csp = db.getEntry(Term.sym("test-1")).getValue().resolve(db);
//		Term call = db.getEntry(Term.sym("org.aiddl.common.reasoning.constraint.csp-solver"), Term.tuple(Term.sym("solve"), csp)).getValue().resolve(db);
//		
//		RequestHandler server = new RequestHandler(new Evaluator(db));
//		CollectionTerm serviceConfigs = db.getEntry(Term.sym("service-configs")).getValue().asCollection();
//		server.loadServices(serviceConfigs);
//		server.satisfyRequest(call, db, exec_module);
//
//	}
	
	public void test4Queens() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/reasoning/constraint/4-queens.aiddl", db, fReg);
		
		CspSolver cspSolver = new CspSolver();
		Term conf_term = Parser.ParseTerm("[eval:org.aiddl.eval]");
		Map<Term,Term> config = conf_term.asList().getMap();
		cspSolver.configure(config , fReg );
		
		((Evaluator)fReg.getFunction(Term.sym("org.aiddl.eval"))).setContainer(db);
		
		
		Term csp = db.getEntry(Term.sym("csp")).getValue().resolve(db);

		Term answer = cspSolver.apply(csp);
		
		assertTrue ( answer.size() == 4 );
	}
	
	public void test3Queens() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/reasoning/constraint/3-queens.aiddl", db, fReg);
		
		CspSolver cspSolver = new CspSolver();
		Term conf_term = Parser.ParseTerm("[eval:org.aiddl.eval]");
		Map<Term,Term> config = conf_term.asList().getMap();
		cspSolver.configure(config , fReg );
		
		((Evaluator)fReg.getFunction(Term.sym("org.aiddl.eval"))).setContainer(db);
		
		
		Term csp = db.getEntry(Term.sym("csp")).getValue().resolve(db);
				
		Term answer = cspSolver.apply(csp);
			
		assertTrue ( answer.equals(CommonTerm.NIL) );
	}
}
