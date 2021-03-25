package org.aiddl.core;

import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestCollection extends TestCase {
		
	@Override
	public void setUp() throws Exception {
	}

	@Override
	public void tearDown() throws Exception {
	}

	public void testCollectionOperations() {
		Term a = Term.sym("a");
		Term b = Term.sym("b");
		Term c = Term.sym("c");
		
		CollectionTerm S1 = Term.set(a,b,c);
		CollectionTerm S2 = Term.set(a,b);
		
		assertTrue( S1.contains(c) );
		assertFalse( S2.contains(c));
		
		assertFalse( S1.equals(S2) );
		
		assertTrue( S1.containsAll(S2) );
		assertFalse( S2.containsAll(S1) );
		assertTrue( S2.containsAny(S1) );
		assertTrue( S1.containsAny(S2) );
		assertTrue( S1.remove(c).equals(S2) );
		
	}

	
}
