package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestKeyValue extends TestCase {
	
	Container db;

	@Override
	public void setUp() throws Exception {
		db = new Container();
	}

	@Override
	public void tearDown() throws Exception {
	}
	
	/**
	 * Fixed bug where symbols with dashes were not parsed correctly.
	 */
	public void testKeyValuePair() {
		Term a = Parser.ParseTerm("(?k : ?v)");
		Term b = Parser.ParseTerm("(() : (2/10 8/10))");
		Substitution s = a.match(b);
		Term t = a.substitute(s);
		t.equals(b);
		
		assertTrue(s.getMap().size() == 2);
		assertTrue(a.substitute(s).equals(b));
	}
	
	/**
	 * Fixed bug where symbols with dashes were not parsed correctly.
	 */
	public void testKeyValuePairComplex() {
		Term kvp = Parser.ParseTerm("a : b : c : d");
		
		
		assertTrue( kvp.getKey().equals(Term.sym("a")) ); 
		assertTrue( kvp.getValue().equals(Parser.ParseTerm("b : c : d")) );
		
		kvp = kvp.getValue();
		
		assertTrue( kvp.getKey().equals(Term.sym("b")) ); 
		assertTrue( kvp.getValue().equals(Parser.ParseTerm("c : d")) );
		
		kvp = kvp.getValue();
		
		assertTrue( kvp.getKey().equals(Term.sym("c")) ); 
		assertTrue( kvp.getValue().equals(Parser.ParseTerm("d")) );
	}
	
	public void testImplicitMap() {
		TupleTerm m = (TupleTerm) Parser.ParseTerm("(a : b, c : d, e : f)");
		
		assertTrue( m.get(Term.sym("a")).equals(Term.sym("b")) );
		assertTrue( m.get(Term.sym("c")).equals(Term.sym("d")) );
		assertTrue( m.get(Term.sym("e")).equals(Term.sym("f")) );
	}
	
	public void testParsedKeyValueTerm() {
		Term t = Parser.ParseTerm("x:v");
		assertTrue( t instanceof KeyValueTerm );
	}
		
}
