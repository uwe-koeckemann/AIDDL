package org.aiddl.common.java.math.graph;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.HashMap;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.Global;
import org.aiddl.core.java.function.FunctionRegistry;

public class UndirectedGraph2Dot implements ConfigurableFunction {

	private String exportFilename = Global.workDir() + "output.dot";
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
		sB.append("graph {\n");
		Map<Term, Integer> nodeMap = new HashMap<>();
		int next_free = 0;
		
		SetTerm weights = null; 
		if ( G.asTuple().containsKey(GraphTerm.Weights) ) {
			weights = G.get(GraphTerm.Weights).asSet();
		}
		SetTerm labels = null; 
		if ( G.asTuple().containsKey(GraphTerm.Labels) ) {
			labels = G.get(GraphTerm.Labels).asSet();
		}
		
		Term attributes = G.get(GraphTerm.Attributes);
		
		for ( Term u : G.get(GraphTerm.Nodes).asCollection() ) {
			Term node_atts = attributes.get(u);
			nodeMap.put(u, (++next_free));
			
			String additional = "";
			if ( node_atts != null && node_atts.asCollection().containsKey(Term.sym("pos")))  {
				Term pos = node_atts.get(Term.sym("pos")); 
				additional += String.format(", pos=\"%d,%d!\"", pos.get(0).getIntValue(), pos.get(1).getIntValue());
			}
			String label = u.toString();
			if ( node_atts != null && node_atts.asCollection().containsKey(Term.sym("label")))  {
				if ( node_atts.get(Term.sym("label")).equals(CommonTerm.NIL) ) {
					label = "";
				} else {
					label = node_atts.get(Term.sym("label")).toString();
				}
			}
			
			if ( node_atts != null && node_atts.asCollection().containsKey(Term.sym("shape")))  {
				additional += String.format(", shape=\"%s\"", node_atts.get(Term.sym("shape")).toString() );
			}			
			
			sB.append(String.format("  n%d [label=\"%s\"%s];\n", next_free, label, additional));
		}
		for ( Term e : G.get(GraphTerm.Edges).asCollection() ) {
//			Term e_nodes = e.get(GraphTerm.Edge);
			
			ListTerm L = Term.list(e.getSetCopy());
			
			
			Term source = L.get(0);
			Term dest = L.get(1);
			
			String config = "";

			String labelStr = "";
			if (  weights != null ) {
				Term weight = weights.get(e);
				if ( weight != null ) {
					labelStr += weight.toString();	
				}
				
			}
			if ( labels != null ) {
				Term label = weights.get(e);
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
			
			sB.append( String.format("  n%d -- n%d%s;\n", nodeMap.get(source), nodeMap.get(dest), config));
			
		}
		sB.append("}");
				
		return sB.toString();
	}
}
