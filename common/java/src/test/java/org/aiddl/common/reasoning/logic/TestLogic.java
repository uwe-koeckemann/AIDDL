package org.aiddl.common.reasoning.logic;

import java.util.Map;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.reasoning.constraint.CspSolver;
import org.aiddl.core.container.Container;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;
import org.aiddl.core.unit_test.RunTests;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
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
