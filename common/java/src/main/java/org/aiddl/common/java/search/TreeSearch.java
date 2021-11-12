package org.aiddl.common.java.search;

import java.util.LinkedList;
import java.util.Map;
import java.util.Stack;

import org.aiddl.common.java.CommonTerm;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.representation.ListTerm;
import org.aiddl.core.java.representation.Term;
import org.aiddl.core.java.tools.Logger;

public class TreeSearch implements Function, ConfigurableFunction {

	LinkedList<Term> choice = new LinkedList<>();
	Stack<ListTerm> search_space = new Stack<>();  
	Stack<Integer> search_idx = new Stack<>();  
	
	static Term Expand = Term.sym("expand");
	static Term Next = Term.sym("next");
	
	private String loggerName = "TreeSearch";
	private int verbose = 0;

	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.loggerName = settings.getOrDefault(Term.sym("log-name"), Term.string(this.loggerName)).toString();
		this.verbose = settings.getOrDefault(Term.sym("verbose"), Term.integer(0)).getIntValue();
	}	
					
	@Override
	public Term apply(Term args) {

		Term operator = args.get(0);
		Term r = CommonTerm.NIL;
		
		if ( operator.equals(Expand) ) {
			ListTerm expansion = args.get(1).asList();
			if ( verbose >= 1 ) {
				Logger.msg(this.loggerName, "Expansion: " + expansion);
			}
			search_space.push(expansion);
			search_idx.push(-1);
			choice.add(CommonTerm.NIL);
			r = next();
		} else if ( operator.equals(Next) ) {
			Term n = next();
			if ( verbose >= 1 ) {
				Logger.msg(this.loggerName, "Next: " + n);
			}
			return n;
		}
		return r;
	}
	
	private Term next() {
		Term r; 
		Integer idx = null;
		while ( !search_idx.isEmpty() && (idx = (search_idx.pop()+1)) >= search_space.peek().size() ) {
			search_space.pop();
			choice.removeLast();
		}
		if ( search_space.isEmpty() ) {
			r = CommonTerm.NIL;
		} else {
			choice.removeLast();
			search_idx.push(idx);
			choice.addLast(search_space.peek().get(idx));
			r = Term.list(choice);
		}
		return r;
	}
}
