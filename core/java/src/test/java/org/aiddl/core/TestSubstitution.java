package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestSubstitution extends TestCase {
	
	Container db;
	
	@Override
	public void setUp() throws Exception {
		db = new Container();
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	public void testSubstitutionOnVariable() {
		Term X = Term.var("X");
		Term a = Term.sym("a");
		Substitution s = new Substitution();
		s.add(X, a);
		assertTrue( X.substitute(s).equals(a) );
	}
}
