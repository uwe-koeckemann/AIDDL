package org.aiddl.common.planning.state_variable.heuristic;

import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.core.container.Container;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestMaxCost extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testSumCostHeuristicValue01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-01.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		SumCostHeuristic maxCost = new SumCostHeuristic();
		
		maxCost.initialize(Pi);		
		assertTrue( maxCost.compute(s0, goal).equals(Term.integer(6)) );
	}
	
	public void testSumCostHeuristicValue02() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-02.aiddl", db, fReg);
				
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		SumCostHeuristic maxCost = new SumCostHeuristic();
		
		maxCost.initialize(Pi);
		
		assertTrue( maxCost.compute(s0, goal).equals(Term.integer(3)) );
	}
	
	public void testSumCostHeuristicValue03() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		SumCostHeuristic maxCost = new SumCostHeuristic();
		maxCost.initialize(Pi);
				
		assertTrue( maxCost.compute(s0, goal).isInfPos() );
	}
	
	public void testSumCostHeuristicValue04() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-04.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();

		SumCostHeuristic maxCost = new SumCostHeuristic();
		
		maxCost.initialize(Pi);
		
		assertTrue( maxCost.compute(s0, goal).equals(Term.integer(0)) );
	}
}
