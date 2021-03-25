package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestSymbolic extends TestCase {
	
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
	
	public void testSymbolic() {
		Parser.parseString("(#mod self tc) (Term x a)", db, fReg);
		Entry e = db.getEntry(Term.sym("x"));
		
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		assertTrue(e.getValue() instanceof SymbolicTerm);
		assertTrue(e.getType().equals(Term.sym("Term")));
		assertTrue(e.getName().equals(Term.sym("x")));
		assertTrue(e.getValue().equals(Term.sym("a")));
	}

	/**
	 * Fixed bug where symbols with dashes were not parsed correctly.
	 */
	public void testSymbolWithDashes() {
		Term x = Parser.ParseTerm("key-value-pair");
		assertTrue(x instanceof SymbolicTerm);
		assertTrue(x.toString().equals("key-value-pair"));
	
		Term t = Parser.ParseTerm("(type self key-value-pair)");
		assertTrue(t.get(2).toString().equals("key-value-pair"));
	}
}
