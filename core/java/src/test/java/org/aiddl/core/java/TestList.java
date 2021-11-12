package org.aiddl.core.java;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestList extends TestCase {
	
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
	
	
	public void testListBase() {
		Term L = Term.list(Term.sym("a"), Term.sym("b"), Term.sym("c"), Term.sym("c"));
		
		assertTrue(L.size() == 4);
	}

	public void testList() {
		Parser.parseString("(#mod s tc) (#list L [0 1 2 3 4 5 6 7 8 9])", db, fReg);
		
		Entry e = db.getEntry(Term.sym("L"));
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		
		assertTrue(e.getValue() instanceof ListTerm);
		assertTrue(e.getType().equals(Term.sym("#list")));
		assertTrue(e.getName().equals(Term.sym("L")));
		assertTrue(e.getValue().equals(
				Term.list(
						Term.integer(0), 
						Term.integer(1), 
						Term.integer(2),
						Term.integer(3), 
						Term.integer(4), 
						Term.integer(5), 
						Term.integer(6), 
						Term.integer(7), 
						Term.integer(8), 
						Term.integer(9)))); 
		assertTrue(e.getValue().equals(Parser.ParseTerm( "[ 0 1 2 3 4 5 6 7 8 9 ]")));
		
		assertFalse(e.getValue().equals(Parser.ParseTerm("[ 0 1 2 3 4 5 6 7 8 ]")));
		assertFalse(e.getValue().equals(Parser.ParseTerm("[ 0 1 2 3 4 5 6 7 8 9 10]")));
		assertFalse(e.getValue().equals(Parser.ParseTerm("[ 0 1 2 3 4 5 6 7 8 0]")));
		assertFalse(e.getValue().equals(Parser.ParseTerm("[ ]")));
	}
	


}
