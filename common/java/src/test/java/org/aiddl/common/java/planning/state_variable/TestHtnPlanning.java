package org.aiddl.common.java.planning.state_variable;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.common.java.planning.PlanningTerm;
import org.aiddl.common.java.planning.task_network.TotalOrderForwardDecomposition;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.LockableList;
import org.aiddl.core.java.tools.Logger;
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
		FunctionRegistry fReg = db.getFunctionRegistry();

		Parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-01.aiddl", db);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
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
		FunctionRegistry fReg = db.getFunctionRegistry();

		Parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-02.aiddl", db);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);

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
		FunctionRegistry fReg = db.getFunctionRegistry();

		Parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-03.aiddl", db);
		Logger.addPrintStream(System.out);
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
				
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
