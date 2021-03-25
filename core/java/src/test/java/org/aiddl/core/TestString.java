package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.StringTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestString extends TestCase {
	
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
	
	public void testString() {
		Parser.parseString("(#mod s tc) (String msg \"Hello Dave.\")", db, fReg);
		Entry e = db.getEntry(Term.sym("msg"));
		
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		assertTrue(e.getValue() instanceof StringTerm);
		assertTrue(e.getType().equals(Term.sym("String")));
		assertTrue(e.getName().equals(Term.sym("msg")));
		assertTrue(e.getValue().equals(Term.string("\"Hello Dave.\"")));
		assertTrue(e.getValue().equals(Term.string("Hello Dave.")));
	}
}
