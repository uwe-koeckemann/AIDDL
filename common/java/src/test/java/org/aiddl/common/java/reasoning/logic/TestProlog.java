package org.aiddl.common.java.reasoning.logic;

import org.aiddl.common.java.reasoning.logic.prolog.PrologQueryRunner;
import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestProlog extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	String aiddlTestStr = "../test/"; // System.getenv("AIDDL_TEST");
	
	public void testQuery() {
		Container db = new Container();
		Parser.parseFile(aiddlTestStr + "/reasoning/logic/prolog/test.aiddl", db);
		Evaluator eval = db.evaluator();
	
		PrologQueryRunner qRunner = new PrologQueryRunner();
		//qRunner.setWorkDir();
		
		Term kb = db.getEntry(Term.sym("KB")).getValue();
		Term query = db.getEntry(Term.sym("query")).getValue();
		
		
		Term arg = Term.tuple(
				Term.keyVal(LogicTerm.KB, kb),
				Term.keyVal(LogicTerm.Query, query)	);
		arg = eval.apply(arg);	
		
		assertTrue( eval.apply( db.getFunctionRegistry().getInputChecker(qRunner.getInterfaceUri()), arg ).getBooleanValue() );
		Term answer = qRunner.apply(arg);
		assertTrue( eval.apply( db.getFunctionRegistry().getOutputChecker(qRunner.getInterfaceUri()), answer ).getBooleanValue() );
		
		assertTrue ( answer.size() == 2 );
	}
}
