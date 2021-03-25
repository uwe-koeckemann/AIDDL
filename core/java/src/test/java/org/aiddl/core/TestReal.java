package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.RealTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestReal extends TestCase {
	
	Container db;
	FunctionRegistry fReg;
	
	@Override
	public void setUp() throws Exception {
		db = new Container();
		fReg = DefaultFunctions.createDefaultRegistry(db);
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testReal() {
		Parser.parseString("(#mod s tc) (real PI 3.14)", db, fReg);
		Entry e = db.getEntry(Term.sym("PI"));
		
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		assertTrue(e.getValue() instanceof RealTerm);
		assertTrue(e.getType().equals(Term.sym("real")));
		assertTrue(e.getName().equals(Term.sym("PI")));
		assertTrue(e.getValue().equals(Term.real(3.14)));
		
		Parser.ParseEntryIntoDatabase("(real x -1.2)", db, fReg);
	}
}
