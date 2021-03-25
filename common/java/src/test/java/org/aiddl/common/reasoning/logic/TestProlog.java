package org.aiddl.common.reasoning.logic;

import org.aiddl.common.reasoning.logic.LogicTerm;
import org.aiddl.common.reasoning.logic.prolog.PrologQueryRunner;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;

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
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile(aiddlTestStr + "/reasoning/logic/prolog/test.aiddl", db, fReg);
		Evaluator eval = (Evaluator) fReg.getFunction(DefaultFunctions.EVAL);
	
		PrologQueryRunner qRunner = new PrologQueryRunner();
		
		Term kb = db.getEntry(Term.sym("KB")).getValue();
		Term query = db.getEntry(Term.sym("query")).getValue();
		
		
		Term arg = Term.tuple(
				Term.keyVal(LogicTerm.KB, kb),
				Term.keyVal(LogicTerm.Query, query)	);
		arg = eval.apply(arg);	
		
		assertTrue( eval.apply( fReg.getInputChecker(qRunner.getInterfaceUri()), arg ).getBooleanValue() );
		Term answer = qRunner.apply(arg);
		assertTrue( eval.apply( fReg.getOutputChecker(qRunner.getInterfaceUri()), answer ).getBooleanValue() );
		
		assertTrue ( answer.size() == 2 );
	}
}
