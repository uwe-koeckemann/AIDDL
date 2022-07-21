package org.aiddl.common.java.planning.state_variable.heuristic;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.java.math.graph.Graph;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.common.java.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.java.planning.state_variable.data.CausalGraphCreator;
import org.aiddl.common.java.planning.state_variable.data.DomainTransitionGraphCreator;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
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
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-01.aiddl", db);
		
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
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-02.aiddl", db);
		
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

		Parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl", db);
		
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
		
		Parser.parseFile("../test/planning/state-variable/elevator/problem-04.aiddl", db);
		
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
