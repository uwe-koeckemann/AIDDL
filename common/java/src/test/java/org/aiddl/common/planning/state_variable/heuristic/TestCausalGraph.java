package org.aiddl.common.planning.state_variable.heuristic;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.planning.state_variable.data.CausalGraphCreator;
import org.aiddl.common.planning.state_variable.data.DomainTransitionGraphCreator;
import org.aiddl.common.planning.state_variable.heuristic.CausalGraphHeuristic;
import org.aiddl.core.container.Container;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestCausalGraph extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testCausalGraphHeuristicValue01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-01.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		CausalGraphCreator causalGraphComp = new CausalGraphCreator();
		DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
		CausalGraphHeuristic cgComp = new CausalGraphHeuristic();
		
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();
		
		Term A = groundOps.apply(Pi);
	
		Graph CG = new Graph( causalGraphComp.apply(A) );
		
		Term DTGs_term = dtgComp.apply(A);
		Map<Term,Graph> DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}
		cgComp.setDataStructures(CG, DTGs);
		assertTrue( cgComp.compute(s0, goal).equals(Term.integer(8)) );
	}
	
	public void testCausalGraphHeuristicValue02() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-02.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
//		Term O = db.getEntry(Term.sym("O")).getValue();
		
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		CausalGraphCreator causalGraphComp = new CausalGraphCreator();
		DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
		CausalGraphHeuristic cgComp = new CausalGraphHeuristic();
		
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();
		
		Term A = groundOps.apply(Pi);
	
		Graph CG = new Graph( causalGraphComp.apply(A) );
		
	
		Term DTGs_term = dtgComp.apply(A);
		Map<Term,Graph> DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}
		
		cgComp.setDataStructures(CG, DTGs);
		assertTrue( cgComp.compute(s0, goal).equals(Term.integer(4)) );
	}
	
	public void testCausalGraphHeuristicValue03() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
//		Term O = db.getEntry(Term.sym("O")).getValue();
		
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		CausalGraphCreator causalGraphComp = new CausalGraphCreator();
		DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
		CausalGraphHeuristic cgComp = new CausalGraphHeuristic();
		
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();
		
		Term A = groundOps.apply(Pi);
	
		Graph CG = new Graph( causalGraphComp.apply(A) );
		
		Term DTGs_term = dtgComp.apply(A);
		Map<Term,Graph> DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}
		
		cgComp.setDataStructures(CG, DTGs);
		assertTrue( cgComp.compute(s0, goal).isInfPos() );
	}
	
	public void testCausalGraphHeuristicValue04() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-04.aiddl", db, fReg);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
//		Term O = db.getEntry(Term.sym("O")).getValue();
		
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		CausalGraphCreator causalGraphComp = new CausalGraphCreator();
		DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
		CausalGraphHeuristic cgComp = new CausalGraphHeuristic();
		
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();
		
		Term A = groundOps.apply(Pi);
	
		Graph CG = new Graph( causalGraphComp.apply(A) );
		
		Term DTGs_term = dtgComp.apply(A);
		Map<Term,Graph> DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}
		
		cgComp.setDataStructures(CG, DTGs);
		assertTrue( cgComp.compute(s0, goal).equals(Term.integer(0)) );
	}
}
