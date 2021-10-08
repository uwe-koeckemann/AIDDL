package org.aiddl.util;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.unit_test.RunTests;

import junit.framework.TestCase;
import org.aiddl.util.function.RegistryLoader;

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
		assertTrue( RunTests.testFile("../test/function/math.aiddl", freg) );
	}
	
	public void testRandom() {
		Logger.addPrintStream(System.out);
		assertTrue(RunTests.testFile("../test/function/random.aiddl", freg));
	}
}
