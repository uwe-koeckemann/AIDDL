package org.aiddl.common.java.math.graph;

import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class GraphTerm {
	public static SymbolicTerm NIL = Term.sym("NIL");
	
	public static SymbolicTerm Graph = Term.sym("graph");
	
	public static SymbolicTerm Edges = Term.sym("E");
	public static SymbolicTerm Nodes = Term.sym("V");
	
	public static SymbolicTerm Weights = Term.sym("weights");
	public static SymbolicTerm Labels = Term.sym("labels");
	
	public static SymbolicTerm Attributes = Term.sym("node-attributes");
}
