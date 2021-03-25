package org.aiddl.core;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;

import junit.framework.TestCase;

@SuppressWarnings("javadoc")
public class TestTuple extends TestCase {
	
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


	public void testTuple() {
		Parser.parseString("(#mod s tc) (PlanningProblem p1 (S@tc G@tc O@tc))", db, fReg);
		Entry e = db.getEntry(Term.sym("p1"));
		
		assertTrue(e.getType() instanceof SymbolicTerm);
		assertTrue(e.getName() instanceof SymbolicTerm);
		assertTrue(e.getValue() instanceof TupleTerm);
		assertTrue(e.getType().equals(Term.sym("PlanningProblem")));
		assertTrue(e.getName().equals(Term.sym("p1")));
		assertTrue(e.getValue().equals(
				Term.tuple(
						Term.ref(Term.sym("S"), Term.sym("tc")), 
						Term.ref(Term.sym("G"), Term.sym("tc")), 
						Term.ref(Term.sym("O"), Term.sym("tc")))));
		assertTrue(e.getValue().equals(Parser.ParseTerm("(S@tc G@tc O@tc)")));
		
		assertFalse(e.getValue().equals(
				Term.tuple(
						Term.ref(Term.sym("S"), Term.sym("tc")), 
						Term.ref(Term.sym("G1"), Term.sym("tc")), 
						Term.ref(Term.sym("O"), Term.sym("tc")))));
	}
	
	public void testNamedTuple() {
		Term k1 = Term.sym("key1");
		Term k2 = Term.sym("key2");
		Term k3 = Term.sym("key3");
		Term v1 = Term.sym("value1");
		Term v2 = Term.sym("value2");
		Term v3 = Term.sym("value3");
		Term e1 = Term.sym("extra1");
		Term e2 = Term.sym("extra2");
		Term e3 = Term.sym("extra3");
		
		TupleTerm t = Term.tuple(Term.keyVal(k1, v1), Term.keyVal(k2, v2), e1, Term.keyVal(k3, v3), e2, e3);
		
		assertTrue( t.get(k1).equals(v1) );
		assertTrue( t.get(k2).equals(v2) );
		assertTrue( t.get(k3).equals(v3) );
		
		assertTrue( t.get(0).equals(Term.keyVal(k1, v1)) );
		assertTrue( t.get(1).equals(Term.keyVal(k2, v2)) );
		assertTrue( t.get(2).equals(e1) );
		assertTrue( t.get(3).equals(Term.keyVal(k3, v3)) );
		assertTrue( t.get(4).equals(e2) );
		assertTrue( t.get(5).equals(e3) );
		
	}
}
