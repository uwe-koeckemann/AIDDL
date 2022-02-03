package org.aiddl.common.java.math.graph;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.HashMap;

import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.function.FunctionRegistry;

public class DirectedGraph2Dot implements ConfigurableFunction {

	private String exportFilename = "output.dot";
	boolean verbose = true;
	
	public void setExportFilename ( String s ) {
		this.exportFilename = s;
	}
	
	@Override
	public void configure(Map<Term, Term> settings , FunctionRegistry fReg) {
		this.verbose = Term.sym("true").equals(settings.get(Term.sym("verbose")));
		if ( settings.containsKey( Term.sym("exportFilename") ) ) {
			this.exportFilename = settings.get(Term.sym("exportFilename")).toString();
		}
	}

	@Override
	public Term apply( Term G ) {
		String s = extract(G);	
		BufferedWriter out = null;

		try {
		    FileWriter fstream = new FileWriter(exportFilename); 
		    out = new BufferedWriter(fstream);
		    out.write(s);
		    out.close();
		} catch (IOException e) {
		    e.printStackTrace();
		} 
		return Term.string(s);
	}
		
	public String extract( Term G ) {
		StringBuilder sB = new StringBuilder();
		sB.append("digraph {\n");
		Map<Term, Integer> nodeMap = new HashMap<>();
		int next_free = 0;
		
		SetTerm weights = null; 
		if ( G.asTuple().containsKey(GraphTerm.Weights) ) {
			G.get(GraphTerm.Weights).asSet();
		}
		SetTerm labels = null; 
		if ( G.asTuple().containsKey(GraphTerm.Labels) ) {
			labels = G.get(GraphTerm.Labels).asSet();
		}
		
		for ( Term u : G.get(GraphTerm.Nodes).asCollection() ) {
			nodeMap.put(u, (++next_free));
			sB.append(String.format("  n%d [label=\"%s\"];\n", next_free, u.toString()));
		}
		for ( Term e : G.get(GraphTerm.Edges).asCollection() ) {
			Term source = e.get(0);
			Term dest = e.get(1);
			String config = "";
			String labelStr = "";

			if (  weights != null ) {
				Term weight = weights.get(e);
				if ( weight != null ) {
					labelStr += weight.toString();	
				}
			}
			if ( labels != null ) {
				Term label = labels.get(e);
				if ( label != null ) {
					if ( labelStr != "" ) {
						labelStr += ", ";
					}
					labelStr += label.toString();
				}
			}
			if ( labelStr != "" ) {
				config += String.format("label=\"%s\"", labelStr);
			}
			
			if ( config != "" ) {
				config = "[" + config + "]";
			}
			sB.append( String.format("  n%d -> n%d%s;\n", nodeMap.get(source), nodeMap.get(dest), config));
		}
		sB.append("}");
				
		return sB.toString();
	}
}
