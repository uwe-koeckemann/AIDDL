package org.aiddl.common.java;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.Function;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestCommon extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testSetOfSets() {
		Container db = new Container();
		FunctionRegistry fReg = DefaultFunctions.createDefaultRegistry(db);
		Parser.parseFile("../test/common.aiddl", db, fReg);
	
		Evaluator eval = (Evaluator) fReg.getFunction(Uri.EVAL);
		
		Term def = db.getEntry(Term.sym("set-of-set")).getValue();
		def = eval.apply(def);
		
		Function f = def.asFunRef().getFunction();
		
		assertTrue( f.apply(Term.set(Term.set(Term.integer(3)))).getBooleanValue() );
	}
}
