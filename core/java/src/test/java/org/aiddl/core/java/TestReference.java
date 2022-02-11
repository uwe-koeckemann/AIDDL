package org.aiddl.core.java;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.container.Entry;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.ReferenceTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestReference extends TestCase {
	
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

	public void testReference() {
		Parser.parseString("(#mod s tc) (Ref r S@tc)", db, fReg);
		Entry e = db.getEntry(Term.sym("r"));
		
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		assertTrue(e.getValue() instanceof ReferenceTerm);
		assertTrue(e.getType().equals(Term.sym("Ref")));
		assertTrue(e.getName().equals(Term.sym("r")));
		assertTrue(e.getValue().equals(Term.ref(Term.sym("S"), Term.sym("tc"))));
	}
	
	public void testReferenceWithAlias() {
		Parser.parseString("(#mod s tc) (Ref r R@tc)", db, fReg);
		Entry e = db.getEntry(Term.sym("r"));
		
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		assertTrue(e.getValue() instanceof ReferenceTerm);
		assertTrue(e.getType().equals(Term.sym("Ref")));
		assertTrue(e.getName().equals(Term.sym("r")));
		assertTrue(e.getValue().equals(Term.ref(Term.sym("R"), Term.sym("tc"))));
	}
	
}
