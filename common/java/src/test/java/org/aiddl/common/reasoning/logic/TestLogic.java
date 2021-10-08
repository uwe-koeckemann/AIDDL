package org.aiddl.common.reasoning.logic;

import org.aiddl.core.unit_test.RunTests;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestLogic extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}
		
	public void testLogic() {
		assertTrue( RunTests.testFile("../test/reasoning/logic/test-cases.aiddl") );
	}
}
