package org.aiddl.util.java;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.Logger;
import org.aiddl.core.java.unit_test.RunTests;

import junit.framework.TestCase;
import org.aiddl.util.java.function.RegistryLoader;

@SuppressWarnings("javadoc")
public class TestPackages extends TestCase {
	Container db;
	FunctionRegistry freg;

	@Override
	public void setUp() throws Exception {
		db = new Container();
		freg = DefaultFunctions.createDefaultRegistry(db);
		RegistryLoader.register(freg, db);
	}

	@Override
	public void tearDown() throws Exception {
	}

	// TODO: Cannot work until unit tests are moved to util
	/*public void testRequest() {
		Logger.addPrintStream(System.out);
		assertTrue( RunTests.testFile("../test/request/request.aiddl", freg) );
	}*/
	
	public void testMath() {
		Logger.addPrintStream(System.out);
		assertTrue( RunTests.testFile("../test/function/math.aiddl", db, freg) );
	}
	
	public void testRandom() {
		Logger.addPrintStream(System.out);
		assertTrue(RunTests.testFile("../test/function/random.aiddl", db, freg));
	}
}
