package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestSet extends TestCase {
	
	Container db;

	@Override
	public void setUp() throws Exception {
		db = new Container();
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testSetOperations() {
		Term a = Term.sym("a");
		Term b = Term.sym("b");
		Term c = Term.sym("c");
		
		SetTerm S1 = Term.set(a,b,c);
		SetTerm S2 = Term.set(a,b);
		
		assertTrue( S1.contains(c) );
		assertFalse( S2.contains(c));
		
		assertFalse( S1.equals(S2) );
		
		assertTrue( S1.containsAll(S2) );
		assertFalse( S2.containsAll(S1) );
		assertTrue( S2.containsAny(S1) );
		assertTrue( S1.containsAny(S2) );
		assertTrue( S1.remove(c).equals(S2) );
	}
	
	public void testPutAll() {
		SetTerm S_1 = Term.set(Term.keyVal(Term.sym("x"), Term.sym("a")));
		
		SetTerm S_2 = S_1.putAll(Term.set(Term.keyVal(Term.sym("x"), Term.sym("b"))));
		
		assertTrue(S_2.size() == 1);
	}
	
	public void testPutAll2() {
		SetTerm S_1 = (SetTerm) Parser.ParseTerm("{(p a b c) : X}");
		SetTerm S_2 = (SetTerm) Parser.ParseTerm("{(p a b c) : Y}");
		
		SetTerm S_3 = S_1.putAll(S_2);
		
		assertTrue(S_3.size() == 1);
	}
}
