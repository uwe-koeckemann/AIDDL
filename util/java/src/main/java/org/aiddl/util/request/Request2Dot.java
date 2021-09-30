package org.aiddl.util.request;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.ReferenceTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.Global;

public class Request2Dot {
	
	boolean data_nodes = false;
	boolean short_names = true;
	
	private static Map<Term,Term> EmptyMap = new HashMap<>();
	
	private static SymbolicTerm Next = Term.sym("next");
	private static SymbolicTerm True = Term.sym("true");
	private static SymbolicTerm False = Term.sym("false");
	private static SymbolicTerm LoopBack = Term.sym("loop-back");
	private static SymbolicTerm Match = Term.sym("match");	
	
	public void compute( Term r ) {
		LinkedList<Term> startStack = new LinkedList<>();
		LinkedList<Term> endStack = new LinkedList<>();
		Map<Term, Set<Term>> edges = new HashMap<>();
		Map<Term,Map<Term,Term>> labels = new HashMap<>();
		Map<Term,String> node_config = new HashMap<>();
		LinkedList<Term> nodes = new LinkedList<>();
		
		LinkedList<Integer> sub_graph_change = new LinkedList<>();
		
		startStack.add(Term.sym("start"));
		endStack.add(Term.sym("end"));
		
		nodes.add(Term.sym("start"));
		
		
		
		node_config.put(Term.sym("start"), ",style=filled,fillcolor=crimson,shape=Mdiamond");
		node_config.put(Term.sym("end"),   ",style=filled,fillcolor=crimson,shape=Msquare");
		
		int depth = 0;
		sub_graph_change.add(depth);
//		depth++;
		Term inner = process_command(r, edges, labels, nodes, node_config, endStack, sub_graph_change, depth);
		Term first_inner = inner.get(0);
		
		Term end_inner = follow(first_inner, Next, labels);
		
		addEdge(Term.sym("start"), first_inner, edges, labels, Term.sym("next"));
		addEdge(end_inner, Term.sym("end"), edges, labels, Term.sym("next"));
		
		
//		depth--;
		sub_graph_change.add(depth);
//		System.out.println("Final depth: " + depth);
//		System.out.println(sub_graph_change);
		nodes.add(Term.sym("end"));
		
		for ( Term node : nodes ) {
			/**
			 * Follow next trail, find true and next
			 */
			
			if ( node instanceof TupleTerm && node.get(1).get(0).equals(RequestHandler.If) 
			  || node instanceof TupleTerm && node.get(1).get(0).equals(RequestHandler.Match)) {
				Term next_branch = null;
				Term true_end = null;
				Term false_end = null;
				for ( Term v2 : labels.getOrDefault(node, EmptyMap).keySet() ) {
					Term label = labels.get(node).get(v2);
					if ( label.equals(Next) || label.equals(LoopBack) ) {
						next_branch = v2;
					} else if ( label.equals(True) || label.equals(Match) ) {
						true_end = v2;
						true_end = follow(true_end, Next, labels);
					} else if ( label.equals(False) ) {
						false_end = v2;
						false_end = follow(false_end, Next, labels);
					}
				}
				addEdge(true_end, next_branch, edges, labels, labels.get(node).get(next_branch) );
				if ( false_end != null ) {
					addEdge(false_end, next_branch, edges, labels, labels.get(node).get(next_branch) );
				}
			}
		}
				
		Map<Term, Integer> nodeIDs = new HashMap<>();
		Integer c = 0;
		for ( Term v : nodes ) {
			if ( !nodeIDs.containsKey(v) ) {
				c++;
				nodeIDs.put(v, c);
			}
		}
		
		StringBuilder sB = new StringBuilder();
		sB.append("digraph {\n");
		String indent = "  ";
		int i = 0;
		int sub_count = 0;
		for ( Term n : nodes ) {
			if ( i > 0 && sub_graph_change.get(i) > sub_graph_change.get(i-1) ) {
				sB.append(String.format(indent+ "subgraph cluster_%d {\n", sub_count));
				indent += "  ";
				sB.append(indent + "style=filled;\n"); 
				sB.append(indent + String.format("color=lightblue%d;\n", sub_graph_change.get(i)));
				
				sub_count++;
			} else if ( i > 0 && sub_graph_change.get(i) < sub_graph_change.get(i-1) ) {
				for ( int j = 0; j < sub_graph_change.get(i-1) - sub_graph_change.get(i) ; j++ ) {
					indent = indent.substring(2);
					sB.append(indent + "}\n"); 
				}
				
			}
			String config = node_config.getOrDefault(n, "");
			
			String name = n.toString().replace("\"", "");
			if ( n instanceof TupleTerm ) {
				if ( n.get(1).get(0).equals(Term.sym("call")) ) {
					if ( !data_nodes && !short_names ) {
						name = n.get(1).get(1) + "\\n" + n.get(1).get(2) + "\\n" + n.get(1).get(3); 
						name = name.replace("\"", "\\\"");
					} else {
						name = n.get(1).get(1).toString();
					}
				} else if ( n.get(1).get(0).equals(Term.sym("data")) )  {
					name = n.get(1).get(1).toString();
				} else {
					name = n.get(1).toString().replace("\"", "\\\"");	
				}
				
			} 
//			System.out.println(name);
			sB.append(String.format(indent + "  n%d [label=\"%s\"%s];\n", nodeIDs.get(n), name, config));
			i++;
		}
		for ( Term v1 : edges.keySet() ) {
			for ( Term v2 : edges.get(v1) ) {
				if ( v2 != null ) {
				Term label = labels.getOrDefault(v1, EmptyMap).getOrDefault(v2, Term.string(""));

				String label_str = label.toString().replace("\"", "");
				if ( label_str.equals("") ) {
					sB.append( String.format("  n%d -> n%d;\n", nodeIDs.get(v1), nodeIDs.get(v2)) );
				} else {
					String style = "";
					if ( label_str.equals("input") || label_str.equals("output") || label_str.equals("uses") ) {
						style = "style=dashed,";
					}
					sB.append( String.format("  n%d -> n%d ["+style+"label=\"%s\"];\n", nodeIDs.get(v1), nodeIDs.get(v2), label_str) );
				}
				}
			}
		}
		
		sB.append("}");
		
		BufferedWriter out = null;
		try {
		    FileWriter fstream = new FileWriter(Global.workDir() + "/request.dot"); 
		    out = new BufferedWriter(fstream);
		    out.write(sB.toString());
		    out.close();
		} catch (IOException e) {
		    e.printStackTrace();
		} 
	}
	
	private Term follow( Term v, Term label, Map<Term, Map<Term, Term>> labels ) {
		for ( Term v2 : labels.getOrDefault(v, EmptyMap).keySet() ) {
			if ( labels.get(v).get(v2).equals(label) ) {
				return follow(v2, label, labels);
			}
		}
		return v;
	}
	
	private void getAllReferences( Term t, Collection<Term> C ) {
		if ( t instanceof ReferenceTerm ) {
			C.add(t.getRefTarget());
		} else if ( t instanceof CollectionTerm ) {
			for ( Term e : t.asCollection() ) {
				getAllReferences(e, C);
			}
		} else if ( t instanceof TupleTerm ) {
			for ( int i = 0 ;  i < t.size() ; i++ ) {
				getAllReferences(t.get(i), C);
			}
		} else if ( t instanceof KeyValueTerm ) {
			getAllReferences(t.getKey(), C);
			getAllReferences(t.getValue(), C);
		}
	}
	
	private void getOutReferences( Term t, Collection<Term> C  ) {
		if ( t instanceof SymbolicTerm ) {
			C.add(t);
		} else {
			for ( Term e : t.asCollection() ) {
				C.add(e.getValue());
			}
		}
	}
	
	private void addEdge( Term v1, Term v2, Map<Term,Set<Term>> edges ) {
		edges.putIfAbsent(v1, new HashSet<>());
		edges.get(v1).add(v2);
	}
	
	private void addEdge( Term v1, Term v2, Map<Term,Set<Term>> edges, Map<Term, Map<Term,Term>> labels, Term label ) {
		edges.putIfAbsent(v1, new HashSet<>());
		edges.get(v1).add(v2);
		labels.putIfAbsent(v1, new HashMap<>());
		labels.get(v1).put(v2, label);
	}
	
	private Term addNode( Term n, List<Term> nodes, Map<Term, String> node_config, LinkedList<Integer> sub_graph_change, int depth ) {
		Term v_idx = Term.tuple(Term.integer(nodes.size()), n);
		nodes.add(v_idx);
		sub_graph_change.add(depth);
		return v_idx;
	}
	
	private Term addNodeUnique( Term n, List<Term> nodes, Map<Term, String> node_config, LinkedList<Integer> sub_graph_change, int depth ) {
		Term v_idx = Term.tuple(Term.integer(-1), Term.tuple(Term.sym("data"), n));
		nodes.add(v_idx);
		sub_graph_change.add(depth);
		return v_idx;
	}
	
	private Term process_command( Term r, Map<Term,Set<Term>> edges, Map<Term,Map<Term,Term>> labels, List<Term> nodes, Map<Term, String> node_config, LinkedList<Term> end_stack, LinkedList<Integer> sub_graph_change, int depth  ) {
		if ( r instanceof ListTerm ) {
			ListTerm l = (ListTerm)r;
			Term e_prev = null; // start_stack.getFirst();
			Term inner = null;
			Term first_node = null;
			for ( Term e : l ) {
				inner = process_command(e, edges, labels, nodes, node_config, end_stack, sub_graph_change, depth);
				Term first_inner = inner.get(0);
				Term last_inner = inner.get(1);
				if ( first_node == null ) {
					first_node = first_inner;
				}
				if ( e_prev != null ) {
					addEdge(e_prev, first_inner, edges, labels, Term.sym("next"));
				}
				e_prev = first_inner;
			}
			return Term.tuple(first_node, inner.get(0));
		} else {
			if ( r instanceof TupleTerm ) {
				if ( r.get(0).equals(RequestHandler.While) 
						|| r.get(0).equals(RequestHandler.ForAll) ) {
					Term node = addNode(Term.tuple(r.get(0), r.get(1)), nodes, node_config, sub_graph_change, depth);
					depth++;
//					node = addNode(node, nodes, node_config, sub_graph_change, depth);
					node_config.put(node, ",style=filled,fillcolor=darkorange,shape=hexagon");
					
					if ( data_nodes ) {
						HashSet<Term> inputRefs = new HashSet<>();
						getAllReferences(node.get(1).get(1), inputRefs);
						
						for ( Term input : inputRefs ) {
							Term in_node = addNodeUnique(input, nodes, node_config, sub_graph_change, depth);
							addEdge(in_node, node, edges, labels, Term.sym("uses"));
							node_config.put(in_node, ",style=filled,fillcolor=green,shape=cylinder");
						}
					}
					
					Term inner = process_command(r.get(2), edges, labels, nodes, node_config, end_stack, sub_graph_change, depth);
					Term first_inner = inner.get(0);
					Term last_inner = inner.get(1);
					depth--;
					addEdge(node, first_inner, edges, labels, Term.sym("do"));
					addEdge(last_inner, node, edges, labels, LoopBack);
					
					return Term.tuple(node, last_inner);
				}  else if ( r.get(0).equals(RequestHandler.Loop) ) {
//					int idx = 0;
//					for ( Term other_node : nodes ) {
//						if ( other_node instanceof TupleTerm && other_node.get(0).equals(RequestHandler.Loop) ) {
//							idx++;
//						}
//					}
					Term node = Term.tuple(r.get(0));
					depth++;
					node = addNode(node, nodes, node_config, sub_graph_change, depth);
					node_config.put(node, ",style=filled,fillcolor=darkorange,shape=hexagon");
					Term inner = process_command(r.get(1), edges, labels, nodes, node_config, end_stack, sub_graph_change, depth);
					depth--;
					Term first_inner = inner.get(0);
					Term last_inner = inner.get(1);
					
					addEdge(node, first_inner, edges, labels, Term.sym("do"));
					addEdge(last_inner, node, edges, labels, LoopBack);
					return Term.tuple(node, last_inner);
				} else if ( r.get(0).equals(RequestHandler.Match) ) {
					Term node = Term.tuple(r.get(0), r.get(1), r.get(2) );
					node = addNode(node, nodes, node_config, sub_graph_change, depth);
					
					node_config.put(node, ",style=filled,fillcolor=deepskyblue,shape=diamond");

					Term inner = process_command(r.get(3), edges, labels, nodes, node_config, end_stack, sub_graph_change, depth);
					Term first_inner = inner.get(0);
					Term last_inner = inner.get(1);
					addEdge(node, first_inner, edges, labels, Term.sym("match")); // MATCH
				
					return Term.tuple(node, last_inner);
				}  else if ( r.get(0).equals(RequestHandler.If) ) {
					Term node = Term.tuple(r.get(0), r.get(1));
					node = addNode(node, nodes, node_config, sub_graph_change, depth);
					node_config.put(node, ",style=filled,fillcolor=deepskyblue,shape=diamond");
					
					if ( data_nodes ) {
						HashSet<Term> inputRefs = new HashSet<>();
						getAllReferences(node.get(1).get(1), inputRefs);
						
						for ( Term input : inputRefs ) {
							Term in_node = addNodeUnique(input, nodes, node_config, sub_graph_change, depth);
							addEdge(in_node, node, edges, labels, Term.sym("uses"));
							node_config.put(in_node, ",style=filled,fillcolor=green,shape=cylinder");
						}
					}
					
					
					Term inner = process_command(r.get(2), edges, labels, nodes, node_config, end_stack, sub_graph_change, depth);
					Term first_inner = inner.get(0);
					Term last_inner = inner.get(1);
					addEdge(node, first_inner, edges, labels, Term.bool(true)); // TRUE
					
					if ( r.size() > 3 ) {
						Term false_entry = process_command(r.get(3), edges, labels, nodes, node_config, end_stack, sub_graph_change, depth);
						addEdge(node, false_entry, edges, labels, Term.bool(false)); // FALSE						
					}
					
					return Term.tuple(node, last_inner);
				}
			}
		}

		r = addNode(r, nodes, node_config, sub_graph_change, depth);
		node_config.put(r, ",style=filled,fillcolor=gold,shape=box");
		if ( data_nodes ) {
			Term inTerm = null;
			Term outTerm = null;
			if ( r.get(1).get(0).equals(Term.sym("call")) ) {
				inTerm = r.get(1).get(2);
				outTerm = r.get(1).get(3);
			} else if ( r.get(1).get(0).equals(Term.sym("write")) ) {
				inTerm = r.get(1).get(1);
				outTerm = r.get(1).get(2);
			}
	
			HashSet<Term> inputRefs = new HashSet<>();
			getAllReferences(inTerm, inputRefs);
			
			for ( Term input : inputRefs ) {
				Term in_node = addNodeUnique(input, nodes, node_config, sub_graph_change, depth);
				addEdge(in_node, r, edges, labels, Term.sym("input"));
				node_config.put(in_node, ",style=filled,fillcolor=green,shape=cylinder");
			}
			
			HashSet<Term> outputRefs = new HashSet<>();
			getOutReferences(outTerm, outputRefs);
			
			for ( Term output : outputRefs ) {
				Term out_node = addNodeUnique(output, nodes, node_config, sub_graph_change, depth);
				addEdge(r, out_node, edges, labels, Term.sym("output"));
				node_config.put(out_node, ",style=filled,fillcolor=green,shape=cylinder");
			}			
		}
		
		

		return Term.tuple(r, r);
	}
}
