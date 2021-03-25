package org.aiddl.common.reasoning.logic.prolog;

import java.util.HashSet;
import java.util.Set;

import org.aiddl.common.CommonTerm;
import org.aiddl.common.reasoning.logic.LogicTerm;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Term;
import org.aiddl.core.tools.LockableSet;

public class PrologKnowledgeBase implements Function {
	
	private Set<Term> kb = new HashSet<>();
	private Set<Term> assumptions = new HashSet<>();

	PrologQueryRunner qRunner = new PrologQueryRunner();

	@Override
	public Term apply(Term args) {
		Term operator = args.get(0);
		Term r = CommonTerm.NIL;
		
		if ( operator.equals(LogicTerm.ASK ) ) {
			/**
			 * Run a query
			 */
			Term query = args.get(0);
			
			LockableSet kb = new LockableSet();
			kb.addAll(this.kb);
			kb.addAll(assumptions);
			Term answer = qRunner.apply(
					Term.tuple(
							Term.keyVal(LogicTerm.Query, query),
							Term.keyVal(LogicTerm.KB, Term.set(kb))));
			r = answer;
		} else if ( operator.equals(LogicTerm.TELL) ) {
			/**
			 * Add knowledge
			 */
			args.get(1).asCollection().addAllTo(kb);
		} else if ( operator.equals(LogicTerm.FORGET) ) {
			/**
			 * Forget knowledge
			 */
			for ( Term e : args.get(1).asCollection() ) {
				this.kb.remove(e);
			}
		} else if ( operator.equals(LogicTerm.ASSUME) ) {
			/**
			 * Add knowledge to assumptions
			 */
			args.get(1).asCollection().addAllTo(assumptions);
		} else if ( operator.equals(LogicTerm.CLEAR_ASSUMPTIONS) ) {
			/**
			 * Clear all assumptions
			 */
			if ( args.size() > 1 ) {
				for ( Term e : args.get(1).asCollection() ) {
					this.assumptions.remove(e);
				}	
			} else {
				this.assumptions.clear();
			}
		} else {
			throw new IllegalArgumentException("Unsupported operator: " + args);
		}
		
		
		return r;
	}

}
