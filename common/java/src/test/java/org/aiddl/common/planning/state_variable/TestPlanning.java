package org.aiddl.common.planning.state_variable;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.common.math.graph.Graph;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.ForwardSearchPlanner;
import org.aiddl.common.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.planning.state_variable.OperatorDomainEnumerator;
import org.aiddl.common.planning.state_variable.data.CausalGraphCreator;
import org.aiddl.common.planning.state_variable.data.DomainTransitionGraphCreator;
import org.aiddl.common.planning.state_variable.data.RelaxedPlanningGraphCreator;
import org.aiddl.common.planning.state_variable.heuristic.CausalGraphHeuristic;
import org.aiddl.common.planning.temporal.TimelineBasedProblemConverter;
import org.aiddl.core.container.Container;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.Uri;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestPlanning extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");

	public void testCausalGraph() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/elevator/problem-02.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
//		Term O = db.getEntry(Term.sym("O")).getValue();
		
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		CausalGraphCreator causalGraphComp = new CausalGraphCreator();
		DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
		CausalGraphHeuristic cgComp = new CausalGraphHeuristic();

		ForwardSearchPlanner planner = new ForwardSearchPlanner();
		
		SetTerm s0 = Pi.get(PlanningTerm.InitialState).asSet();
		SetTerm goal = Pi.get(PlanningTerm.Goal).asSet();
		
		Term A = groundOps.apply(Pi);
	
		Graph CG = new Graph( causalGraphComp.apply(A) );
		
		Term DTGs_term = dtgComp.apply(A);
		Map<Term,Graph> DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );

		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
	

	
	public void testRelaxedPlanningGraph() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/dock-worker-robot/problem-01.aiddl", db, fReg);
		Parser.parseFile("planning/state-variable/data.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		RelaxedPlanningGraphCreator rpgCreator = new RelaxedPlanningGraphCreator();
		
		Term RPG = rpgCreator.apply(Pi);
		
//		TypeChecker.setVerbose(2);
//		Logger.startLoggingToFile("type-check-fail.org", Logger.OrgTabbing);
//		eval.reserCallCounter();
//		System.out.println(eval.getCallCounter());
//		Logger.stopLoggingToFile("type-check-fail.org");
		
//		System.out.println(Logger.prettyPrint(RPG, 0));
		assertTrue( eval.apply( fReg.getInputChecker( rpgCreator.getInterfaceUri()), Pi  ).getBooleanValue() );
//		eval.setVerbose(2);
		assertTrue( eval.apply( fReg.getOutputChecker(rpgCreator.getInterfaceUri()), RPG ).getBooleanValue() );
	}
	
	public void testFastForward() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/dock-worker-robot/problem-01.aiddl", db, fReg);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
				
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
		
		Map<Term,Term> config = new HashMap<>();
		config.put(Term.sym("heuristic"), Term.sym("fast-forward"));
		planner.configure(config, null);
		
		assertTrue( eval.apply( fReg.getInputChecker( planner.getInterfaceUri()), Pi  ).getBooleanValue() );
		Term pi = planner.apply( Pi );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );
	}
	
	public void testConvertTemporalPlanningToStateVariableAndSolve() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/temporal/elevator/problem-01.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
		Logger.addPrintStream(System.out);
		
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		OperatorReachableEnumerator groundOps = new OperatorReachableEnumerator();
		DomainTransitionGraphCreator dtgComp = new DomainTransitionGraphCreator();
		ForwardSearchPlanner planner = new ForwardSearchPlanner();
		
		SetTerm s0 = Pi.get(Term.sym("constraints")).get(Term.sym("statement")).asSet();
		SetTerm goal = Pi.get(Term.sym("constraints")).get(Term.sym("open-goal")).asSet();
		
		Term problem = Term.tuple(
				Term.keyVal(PlanningTerm.Operators, Pi.get(PlanningTerm.Operators)),
				Term.keyVal(PlanningTerm.InitialState, s0),
				Term.keyVal(PlanningTerm.Goal, goal));
		
		TimelineBasedProblemConverter tbp2svaConf = new TimelineBasedProblemConverter();

		Term svaProblem = tbp2svaConf.apply(problem);
	
		Term A = groundOps.apply(svaProblem);
		
		assertTrue ( A.size() > 0 );
		
		Term DTGs_term = dtgComp.apply(A);
		Map<Term,Graph> DTGs = new HashMap<>();
		for ( Term entry : DTGs_term.asCollection() ) {
			DTGs.put(entry.getKey(), new Graph(entry.getValue()));
		}
		assertTrue( eval.apply( fReg.getInputChecker(planner.getInterfaceUri()), svaProblem ).getBooleanValue() );
		Term pi = planner.apply( svaProblem );
		assertTrue( eval.apply( fReg.getOutputChecker(planner.getInterfaceUri()), pi ).getBooleanValue() );

		assertTrue ( !pi.equals(PlanningTerm.NIL) );
	}
	
	public void testOperatorEnumerator() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/planning/state-variable/dock-worker-robot/problem-01.aiddl", db, fReg);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
				
		Term Pi = db.getEntry(Term.sym("problem")).getValue().resolve(db);
		
		OperatorDomainEnumerator oEnum = new OperatorDomainEnumerator();
		LockableSet O_ground = new LockableSet();
		for ( Term o : Pi.get(PlanningTerm.Operators).asSet() ) {
			Term O_part = oEnum.apply(Term.tuple(o, Pi.get(PlanningTerm.Domains)));
			for ( Term o_g : O_part.asCollection() ) {
				O_ground.add(o_g);
			}
		}
		Term O = Term.set(O_ground);
		assertTrue( O.size() == 148 );
	}
}
