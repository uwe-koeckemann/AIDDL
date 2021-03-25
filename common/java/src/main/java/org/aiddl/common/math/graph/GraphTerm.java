package org.aiddl.common.math.graph;

import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

public class GraphTerm {
	public static SymbolicTerm NIL = Term.sym("NIL");
	
	public static SymbolicTerm Graph = Term.sym("graph");
	
	public static SymbolicTerm Edges = Term.sym("E");
	public static SymbolicTerm Nodes = Term.sym("V");
	
	public static SymbolicTerm Weights = Term.sym("weights");
	public static SymbolicTerm Labels = Term.sym("labels");
	
	public static SymbolicTerm Attributes = Term.sym("node-attributes");
}
