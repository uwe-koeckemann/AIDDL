package org.aiddl.common.math.graph;

import java.util.HashMap;
import java.util.Map;
import org.aiddl.common.CommonTerm;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.LockableList;

public class BellmanFord implements InterfaceImplementation, ConfigurableFunction {
	
	public Map<Term,NumericalTerm> d;
	public Map<Term,Term> pi;
	public FunctionRegistry fReg;
		
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.math.graph.single-source-shortest-path");
	

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg;
	}
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
		
	@Override
	public Term apply( Term args ) {
		TupleTerm G = (TupleTerm) args.get(0);
		Term w = args.get(1);
		Term s = args.get(2);
		
//		Function w_fun = w.asFunRef().getFunction(fReg);
		
		SetTerm V = (SetTerm) G.get(GraphTerm.Nodes);	
		
		initializeSingleSource(G, s);
		for ( int i = 0 ; i < V.asSet().size()-1 ; i++ ) {
			for ( Term e : G.get(GraphTerm.Edges).asSet() ) {
				Term from = e.get(0);
				Term to = e.get(1);
				NumericalTerm weight = Graph.getWeight(w, from, to, fReg);
				relax(from, to, weight);
			}
		}
		for ( Term e : G.get(GraphTerm.Edges).asSet() ) {
//			TupleTerm edge = e.get(GraphTerm.Edge).asTuple();
			Term from = e.get(0);
			Term to = e.get(1);
			NumericalTerm weight = Graph.getWeight(w, from, to, fReg);
			if ( d.get(to).greaterThan(d.get(from).add(weight)) ) {
				LockableList rList = new LockableList();
				rList.add(Term.keyVal(PathKey, GraphTerm.NIL));
				rList.add(Term.keyVal(Distances, GraphTerm.NIL));
				rList.add(Term.keyVal(Predecessors, GraphTerm.NIL));
				return Term.list(rList);
			}
		}

		//pi.put(s, s);
		
		Term[] dMap = new Term[V.size()];
		Term[] piMap = new Term[V.size()];
		int i = 0;
		for ( Term v : V.asSet() ) {
			dMap[i] = Term.keyVal(v, d.get(v));
			piMap[i] = Term.keyVal(v, pi.getOrDefault(v, CommonTerm.NIL));
			i++;
		}
		
		LockableList rList = new LockableList();
		rList.add(Term.keyVal(Distances, Term.list(dMap)));
		rList.add(Term.keyVal(Predecessors, Term.list(piMap)));
		return Term.list(rList);
	}
	
	private final static Term PathKey = Term.sym("path");
	private final static Term Distances = Term.sym("distance");
	private final static Term Predecessors = Term.sym("predecessor");
	
	private void relax( Term u, Term v, NumericalTerm w ) {
		if ( d.get(v).greaterThan(d.get(u).add(w))) {
			d.put(v, d.get(u).add(w));
			pi.put(v, u);
		}
	}

	private void initializeSingleSource( TupleTerm G, Term s ) {
		d = new HashMap<Term, NumericalTerm>();
		pi = new HashMap<Term, Term>();
		SetTerm V = (SetTerm) G.get(GraphTerm.Nodes);
		for ( Term v : V.asSet() ) {
			d.put(v, Term.infPos());
		}
		d.put(s, Term.integer(0));
	}

}
