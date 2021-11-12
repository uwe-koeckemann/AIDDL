package org.aiddl.common.java.reasoning.logic;

import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class LogicTerm {
	public final static SymbolicTerm KB = Term.sym("kb");
	public final static SymbolicTerm Query = Term.sym("query");
	public final static SymbolicTerm CNF = Term.sym("cnf");
	
	public final static SymbolicTerm SAT = Term.sym("sat");
	public final static SymbolicTerm Assignment = Term.sym("assignment");
	
	public final static SymbolicTerm ASK = Term.sym("ask");
	public final static SymbolicTerm TELL = Term.sym("tell");
	public final static SymbolicTerm FORGET = Term.sym("forget");
	public final static SymbolicTerm ASSUME = Term.sym("assume");
	public final static SymbolicTerm CLEAR_ASSUMPTIONS = Term.sym("clear-assumptions");
	
	
}
