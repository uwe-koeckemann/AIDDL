package org.aiddl.common.planning.state_variable;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.planning.PlanningTerm;
import org.aiddl.common.planning.state_variable.OperatorReachableEnumerator;
import org.aiddl.common.planning.task_network.TotalOrderForwardDecomposition;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestHtnPlanning extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testProblem01() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-01.aiddl", db, fReg);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
				
		Term Pi = eval.apply(db.getEntry(Term.sym("problem")).getValue().resolve(db));
		
		OperatorReachableEnumerator opCreator = new OperatorReachableEnumerator();
		Term A = opCreator.apply(Pi);
		
//		Pi = Pi.add(Term.keyVal(PlanningTerm.Operators, A));
		
		LockableList l_new = new LockableList();
		for ( int i = 0 ; i < Pi.size() ; i++ ) {
			if ( !Pi.get(i).getKey().equals(PlanningTerm.Operators) ) {
				l_new.add(Pi.get(i));
			}
		}
		l_new.add(Term.keyVal(PlanningTerm.Operators, A));
		Pi = Term.tuple(l_new);
		
		TotalOrderForwardDecomposition tfd = new TotalOrderForwardDecomposition();
		tfd.initialize(Pi);

		assertTrue( eval.apply( fReg.getInputChecker(tfd.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = tfd.apply(Pi);
		assertTrue( eval.apply( fReg.getOutputChecker(tfd.getInterfaceUri()), pi ).getBooleanValue() );

		
		assertFalse(pi.equals(CommonTerm.NIL));
		
	}
	
	public void testProblem02() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-02.aiddl", db, fReg);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
				
		Term Pi = eval.apply(db.getEntry(Term.sym("problem")).getValue().resolve(db));
		
		OperatorReachableEnumerator opCreator = new OperatorReachableEnumerator();
		Term A = opCreator.apply(Pi);
		
//		Pi = Pi.add(Term.keyVal(PlanningTerm.Operators, A));
		
		LockableList l_new = new LockableList();
		for ( int i = 0 ; i < Pi.size() ; i++ ) {
			if ( !Pi.get(i).getKey().equals(PlanningTerm.Operators) ) {
				l_new.add(Pi.get(i));
			}
		}
		l_new.add(Term.keyVal(PlanningTerm.Operators, A));
		Pi = Term.tuple(l_new);
		
		TotalOrderForwardDecomposition tfd = new TotalOrderForwardDecomposition();
		tfd.initialize(Pi);
		assertTrue( eval.apply( fReg.getInputChecker(tfd.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = tfd.apply(Pi);
		assertTrue( eval.apply( fReg.getOutputChecker(tfd.getInterfaceUri()), pi ).getBooleanValue() );

		assertFalse(pi.equals(CommonTerm.NIL));
		
	}
	
	public void testProblem03() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-03.aiddl", db, fReg);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
				
		Term Pi = eval.apply(db.getEntry(Term.sym("problem")).getValue().resolve(db));
		
		OperatorReachableEnumerator opCreator = new OperatorReachableEnumerator();
		Term A = opCreator.apply(Pi);
		
//		Pi = Pi.add(Term.keyVal(PlanningTerm.Operators, A));
		
		LockableList l_new = new LockableList();
		for ( int i = 0 ; i < Pi.size() ; i++ ) {
			if ( !Pi.get(i).getKey().equals(PlanningTerm.Operators) ) {
				l_new.add(Pi.get(i));
			}
		}
		l_new.add(Term.keyVal(PlanningTerm.Operators, A));
		Pi = Term.tuple(l_new);
		
		TotalOrderForwardDecomposition tfd = new TotalOrderForwardDecomposition();
		tfd.initialize(Pi);

		assertTrue( eval.apply( fReg.getInputChecker(tfd.getInterfaceUri()), Pi ).getBooleanValue() );
		Term pi = tfd.apply(Pi);
		assertTrue( eval.apply( fReg.getOutputChecker(tfd.getInterfaceUri()), pi ).getBooleanValue() );
		
		assertTrue(pi.equals(CommonTerm.NIL));
	}
}
