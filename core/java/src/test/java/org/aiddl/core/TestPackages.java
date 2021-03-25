package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
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
		assertTrue( RunTests.testFile(test_folder + "/test-request.aiddl") );
	}
	
	public void testTypes() {
		Logger.addPrintStream(System.out);
		
		Container db = new Container();
		FunctionRegistry freg = DefaultFunctions.createDefaultRegistry(db);
		Parser.parseFile(test_folder + "/test-types.aiddl", db, freg);
		
		assertTrue( RunTests.testFile(test_folder + "/test-types.aiddl") );		
	}
	
	public void testMath() {
		Logger.addPrintStream(System.out);
		assertTrue( RunTests.testFile("../test/eval/math/test-math.aiddl") );
	}
	
	public void testRandom() {
		Logger.addPrintStream(System.out);
		assertTrue( RunTests.testFile("../test/eval/random/test-random.aiddl") );
	}
	
//	public void testEvalWithInterface() {
//	Container db = new Container();
//	FunctionRegistry freg = DefaultFunctions.createDefaultRegistry(db);
//	
//	db.addModule(Term.sym("test"));
//	db.setEntry(
//			new Entry(
//					Term.sym("#interface"),
//					Term.sym("test-interface"),
//					Parser.ParseTerm("(uri:my.interface "
//							+ "input  : org.aiddl.type.numerical.integer"
//							+ "output : org.aiddl.type.numerical.integer)")));
//
//	freg.loadContainerInterfaces(db);
//	
//	class MyFun implements InterfaceImplementation {
//		@Override
//		public Term compute(Term args) {
//			return args.asNum().mult(Term.integer(13));
//		}
//
//		@Override
//		public SymbolicTerm getInterfaceUri() {
//			return Term.sym("my.interface");
//		}
//	}
//	
//	freg.addFunction(Term.sym("my.fun"), new MyFun());
//	
//	Function f = freg.getFunction(Term.sym("my.interface"));
//	
//	Term r = f.compute(Term.integer(5));
//	assertTrue(Term.integer(65).equals(r));
//}
}
