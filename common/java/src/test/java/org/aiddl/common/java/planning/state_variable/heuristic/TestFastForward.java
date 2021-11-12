package org.aiddl.common.java.planning.state_variable.heuristic;

import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestFastForward extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testFastForwardHeuristicValue01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-01.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		FastForwardHeuristic ff = new FastForwardHeuristic();
		
		ff.initialize(Pi);
		
		assertTrue( ff.compute(s0, goal).equals(Term.integer(8)) );
	}
	
	public void testFastForwardHeuristicValue02() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-02.aiddl", db, fReg);
				
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		FastForwardHeuristic ff = new FastForwardHeuristic();
		
		ff.initialize(Pi);
		
		assertTrue( ff.compute(s0, goal).equals(Term.integer(5)) );
	}
	
	public void testFastForwardHeuristicValue03() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		FastForwardHeuristic ff = new FastForwardHeuristic();
		
		ff.initialize(Pi);
		
		assertTrue( ff.compute(s0, goal).isInfPos() );
	}

	public void testFastForwardHeuristicValue04() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-04.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		FastForwardHeuristic ff = new FastForwardHeuristic();
		
		ff.initialize(Pi);
	
		assertTrue( ff.compute(s0, goal).equals(Term.integer(0)) );
	}
}
