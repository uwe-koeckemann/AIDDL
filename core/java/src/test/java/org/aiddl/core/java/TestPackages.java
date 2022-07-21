package org.aiddl.core.java;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.eval.Evaluator;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.function.Uri;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.unit_test.RunTests;

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
		Evaluator e = db.evaluator();
		e.setVerbose(0);
		Parser.parseFile(test_folder + "/test-types.aiddl", db);

		assertTrue(RunTests.testFile(test_folder + "/test-types.aiddl"));
	}
}
