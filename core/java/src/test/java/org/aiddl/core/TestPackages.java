package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.function.Uri;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.unit_test.RunTests;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestPackages extends TestCase {
	
	Container db;
	String test_folder = "../test";

	
	@Override
	public void setUp() throws Exception {
		db = new Container();
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testEval() {
		Logger.addPrintStream(System.out);
		assertTrue( RunTests.testFile(test_folder + "/test.aiddl") );
	}
	
	public void testTypes() {
		Logger.addPrintStream(System.out);

		Container db = new Container();
		FunctionRegistry freg = DefaultFunctions.createDefaultRegistry(db);
		Evaluator e = (Evaluator) freg.getFunction(Uri.EVAL);
		e.setVerbose(0);
		Parser.parseFile(test_folder + "/test-types.aiddl", db, freg);

		assertTrue(RunTests.testFile(test_folder + "/test-types.aiddl"));
	}
}
