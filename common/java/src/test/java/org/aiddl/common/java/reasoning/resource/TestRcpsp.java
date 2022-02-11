package org.aiddl.common.java.reasoning.resource;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.common.java.reasoning.temporal.allen_constraints.Allen2STP;
import org.aiddl.common.java.reasoning.temporal.simple_temporal.STPSolver;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
import junit.framework.TestCase;
import org.aiddl.core.java.tools.StopWatch;

@SuppressWarnings("javadoc")
public class TestRcpsp extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	

	/*public void testSolvableProblem01() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/reasoning/resource/rcpsp-02.aiddl", db, fReg);
		Term solver_module = Term.sym("org.aiddl.common.reasoning.resource.esta");
		Parser.parseFile(Parser.getModuleFilename(solver_module), db, fReg);
		Term run_module = Term.sym("run-module");		
		db.addModule(run_module);
		
		RequestHandler server = new RequestHandler( fReg );
		server.setEnforceTypeCheck(true);
		
		Term rcpsp = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		Entry main = db.getEntry(solver_module, Term.tuple(Term.sym("main"), rcpsp, run_module));
		
		server.satisfyRequest(main, db, run_module);

		Term answer = db.getEntry(run_module, Term.sym("stp-solution")).getValue();
		
		assertFalse( answer.equals(CommonTerm.NIL) );
	}*/
	
	public void testSolvableProblem01Function() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/reasoning/resource/rcpsp-02.aiddl", db, fReg);

		Term rcpsp = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		EstaScheduler esta = new EstaScheduler();
		esta.initialize(rcpsp);
		
		Term answer = esta.apply(Term.tuple(Term.sym("next")));

		assertFalse( answer.equals(CommonTerm.NIL) );
	}
	
	/*public void testUnsolvableProblem01() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/reasoning/resource/rcpsp-01.aiddl", db, fReg);
		Term solver_module = Term.sym("org.aiddl.common.reasoning.resource.esta");
		Parser.parseFile(Parser.getModuleFilename(solver_module), db, fReg);
		Term run_module = Term.sym("run-module");		
		db.addModule(run_module);
		
		RequestHandler server = new RequestHandler( fReg );

		Term rcpsp = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		Entry main = db.getEntry(solver_module, Term.tuple(Term.sym("main"), rcpsp, run_module));
		server.satisfyRequest(main, db, run_module);		
		Term answer = db.getEntry(run_module, Term.sym("intervals")).getValue();		
		assertTrue( answer.equals(CommonTerm.NIL) );
	}*/

	public void testFlexibilityLoss() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);

		Term data_module    = Parser.parseFile("../test/reasoning/resource/rcpsp-01.aiddl", db, fReg);

		Term rcpsp = db.getEntry(data_module, Term.sym("problem")).getValue();

		MinimalCriticalSetLinearSampler sample = new MinimalCriticalSetLinearSampler();
		FlexibilityLossFunction loss = new FlexibilityLossFunction();
		Allen2STP ac2stp = new Allen2STP();
		STPSolver stp = new STPSolver();

		Term doms = stp.apply(ac2stp.apply(rcpsp.get(ResourceTerm.Constraints)));
		Term peaks = sample.apply(Term.tuple(rcpsp.get(ResourceTerm.Capacity), rcpsp.get(ResourceTerm.Usage), doms));
		/*for ( Term p : peaks.asCollection() ) {
			System.out.println(p + " -> " + loss.apply(Term.tuple(p, doms)));
		}*/
	}
	
	public void testUnsolvableProblem01Function() {
		Logger.addPrintStream(System.out);
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Term data_module    = Parser.parseFile("../test/reasoning/resource/rcpsp-01.aiddl", db, fReg);

		Term rcpsp = db.getEntry(data_module, Term.sym("problem")).getValue();
		
		EstaScheduler esta = new EstaScheduler();

		esta.initialize(rcpsp);
		
		Term answer = esta.apply(Term.tuple(Term.sym("next")));
		assertTrue( answer.equals(CommonTerm.NIL) );
	}
}
