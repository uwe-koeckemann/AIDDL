package org.aiddl.common.java.learning.decision_tree;

import java.util.HashMap;
import java.util.Map;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.interfaces.InterfaceImplementation;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.representation.TupleTerm;

public class DecisionTreeClassifier implements Function, InitializableFunction, InterfaceImplementation {
	
	private final static SymbolicTerm InterfaceUri = Term.sym("org.aiddl.common.learning.supervised.classifier-function");
	
	private Term decisionTree;
	private ListTerm attributes;
	
	@Override
	public SymbolicTerm getInterfaceUri() {
		return InterfaceUri;
	}
	
	@Override
	public void initialize(Term args) {
		decisionTree = args.get(0);
		attributes = args.get(1).asList();
	}
	
	@Override
	public Term apply( Term x ) {
		ListTerm dataPoint = x.asList();
		
		Map<Term,Term> assignment = new HashMap<Term,Term>();
		
		for ( int i = 0 ; i < attributes.size() ; i++ ) {
			assignment.put(attributes.get(i).get(0), dataPoint.get(i));
		}

		return resolveNode(decisionTree, assignment);
	}
	
	private Term resolveNode( Term dt, Map<Term,Term> assignment ) {
		if ( dt instanceof SymbolicTerm ) {
			return dt;
		} else {
			ListTerm decisionList = (ListTerm) dt;
			for ( Term branch : decisionList ) {
				TupleTerm condition = (TupleTerm) branch.get(0);
				if ( assignment.get(condition.get(1)).equals(condition.get(2)) ) {
					return resolveNode(branch.get(1), assignment);
				}
			}
		}
		return Term.sym("FAILURE");
	}

}
