package org.aiddl.common.java.math.graph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.SetTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.LockableSet;

public class DepthFirstSearch implements Function, InterfaceImplementation {
		
	private static final Term WHITE = Term.sym("WHITE");
	private static final Term GRAY  = Term.sym("GRAY");
	private static final Term BLACK = Term.sym("BLACK");
	
	private static final Term NIL   = Term.sym("NIL");
	
	public final static Term PiKey = Term.sym("pi");
	public final static Term DistKey = Term.sym("distances");
	public final static Term FinishKey = Term.sym("finish-times");
	public final static Term ComponentKey = Term.sym("components");
	
	Map<Term,Term> u_color; 
	Map<Term,Term> u_pi;
	Map<Term,Integer> u_d;
	Map<Term,Integer> u_f;
	Integer time;
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.math.graph.depth-first-search");
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}

	@Override
	public Term apply( Term G ) {
		return compute(new Graph(G)); 
	}
		
		
	public Term compute( Graph G ) {
		SetTerm V = G.getGraphTerm().get(GraphTerm.Nodes).asSet();
		
		u_color = new HashMap<>();
		u_pi    = new HashMap<>();
		u_d     = new HashMap<>();
		u_f     = new HashMap<>();
		time	= 0;
		LockableSet visited_groups = new LockableSet();
		
		for ( Term u : V ) {
			u_color.put(u, WHITE);
			u_pi.put(u, NIL);
		}
		for ( Term u : V ) {
			if ( u_color.get(u).equals(WHITE) ) {
				LockableSet visited = DFS_visit(G, u);
				visited_groups.add(Term.set(visited)); 
				
			}
		}
		
		LockableSet pi = new LockableSet();
		LockableSet d = new LockableSet();
		LockableSet f = new LockableSet();
		
		for ( Term u : V ) {
			pi.add(Term.keyVal(u, u_pi.get(u)));
			d.add(Term.keyVal(u, Term.integer(u_d.get(u))));
			f.add(Term.keyVal(u, Term.integer(u_f.get(u))));
		}
		List<Term> rList = new ArrayList<>();
		rList.add(Term.keyVal(PiKey, Term.set(pi)));
		rList.add(Term.keyVal(DistKey, Term.set(d)));
		rList.add(Term.keyVal(FinishKey, Term.set(f)));
		rList.add(Term.keyVal(ComponentKey, Term.set(visited_groups)));
		return Term.list(rList);
	}
	
	private LockableSet DFS_visit( Graph G, Term u ) {
		LockableSet visited = new LockableSet();
		visited.add(u);
		time = time + 1;
		u_d.put(u, time);
		u_color.put(u, GRAY);
		
		for ( Term edge : G.getOutEdges(u) ) {
			Term v = edge.get(1);
			if ( u_color.get(v).equals(WHITE) ) {
				u_pi.put(v,  u);
				visited.addAll( DFS_visit(G, v) );
			}
		}
		u_color.put(u, BLACK);
		time = time + 1;
		u_f.put(u, time);
		return visited;
	}
}
