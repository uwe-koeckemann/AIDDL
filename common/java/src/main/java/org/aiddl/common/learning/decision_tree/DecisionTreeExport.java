package org.aiddl.common.learning.decision_tree;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.tools.Logger;

public class DecisionTreeExport implements ConfigurableFunction {

	private String name = DecisionTreeExport.class.getSimpleName();
	private String exportFilename = "output.dot";
	boolean verbose = true;
	
	public DecisionTreeExport() {
	}
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
		if ( settings.containsKey( Term.sym("exportFilename") ) ) {
			this.exportFilename = settings.get(Term.sym("exportFilename")).toString();
		}
	}

	@Override
	public Term apply( Term dt ) {
		nodeCounter = 0;
		String s = String.format("digraph {\n  n%d [label=\"root\"];\n", nodeCounter);
		s += extract(dt, nodeCounter);
		s += "}";
		
		if ( verbose ) Logger.msg(name, "Generated dot.");
//		System.out.println(s);
		
		BufferedWriter out = null;

		try {
		    FileWriter fstream = new FileWriter(exportFilename); //true tells to append data.
		    out = new BufferedWriter(fstream);
		    out.write(s);
		    out.close();
		} catch (IOException e) {
		    e.printStackTrace();
		} 
		return null;
	}
	
	private int nodeCounter = 0;
	
	private String extract( Term tree, int parentID ) {
		if ( !(tree instanceof ListTerm) ) {
			nodeCounter++;
			String s = String.format("  n%d [label=\"%s\"];\n", nodeCounter, tree.toString());
			s += String.format("  n%d -> n%d;\n", parentID, nodeCounter);
			return s;
		} else {
			String s = "";
			ListTerm decisions = (ListTerm) tree;
			for ( Term decision : decisions ) {
				TupleTerm condition = (TupleTerm) decision.get(0);
				Term subTree = decision.get(1);
				
				nodeCounter++;
				
				s += String.format("  n%d [label=\"%s\"];\n", nodeCounter, condition.toString());
				s += String.format("  n%d -> n%d;\n", parentID, nodeCounter);
				
				s += extract(subTree, nodeCounter);
			}
			return s;
		}
	}
}
