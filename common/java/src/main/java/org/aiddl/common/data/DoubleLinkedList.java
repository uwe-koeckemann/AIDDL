package org.aiddl.common.data;

import java.util.LinkedList;

import org.aiddl.common.CommonTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;

public class DoubleLinkedList implements Function {

	LinkedList<Term> L = new LinkedList<>();
	
	Term PushLeft = Term.sym("push-left");
	Term PushRight = Term.sym("push-right");
	
	Term PopLeft = Term.sym("pop-left");
	Term PopRight = Term.sym("pop-right");
	
	
	@Override
	public Term apply(Term args) {

		Term operator = args.get(0);
		Term arg = args.get(1);
		
		Term r = CommonTerm.NIL;
		
		if ( operator.equals(PushLeft) ) {
			L.addFirst(arg);
		} else if ( operator.equals(PushRight) ) {
			L.addLast(arg);
		} else if ( operator.equals(PopLeft) ) {
			r = L.pollFirst();
		} else if ( operator.equals(PopRight) ) {
			r = L.pollLast();
		} 
		
		return r;
	}

}
