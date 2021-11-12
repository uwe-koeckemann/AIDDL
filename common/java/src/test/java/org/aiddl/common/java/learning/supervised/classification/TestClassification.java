package org.aiddl.common.java.learning.supervised.classification;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestClassification extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testProblemType() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		
		Parser.parseFile("../test/learning/classification/problem-01.aiddl", db, fReg).asSym();

		Entry problemEntry = db.getEntry(Term.sym("problem"));
		
		Term problem = problemEntry.getValue();
		Function typeCheck = problemEntry.getType().asFunRef().getFunction();
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		problem = eval.apply(problem);
		Term r = typeCheck.apply(problem);
		
		assertTrue(r.getBooleanValue());
	}
	
}
