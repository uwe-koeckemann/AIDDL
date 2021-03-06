package org.aiddl.core.function.type;

import org.aiddl.core.function.Evaluator;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.Logger;

public class TypeChecker implements Function {
	
	Evaluator eval;
	Term def;

	public TypeChecker( Term typeDef, Evaluator eval ) {
		this.def = typeDef;
		this.eval = eval;
	}
	
	@Override
	public Term apply( Term t ) {
		return Term.bool(this.check(this.def, t));
	}
	
	private boolean check( Term type, Term t ) {
		//Logger.msg("TypeCheck", t + " ?? " + type);
		boolean r;
		
		if ( type instanceof TupleTerm ) {
			Term typeClass = type.get(0);
			
			if ( typeClass.equals(Term.sym("basic-type")) ) {
//				Term e = Term.tuple(Term.sym("org.aiddl.eval.type"), t, Term.tuple(type.get(1), Term.sym("#self")));
				Term e = Term.tuple(Term.tuple(type.get(1), t));
				r = this.eval.apply(e).getBooleanValue();

			} else if ( typeClass.equals(Term.sym("set-of")) ) {
				Term subType = type.get(1);
				if ( t instanceof SetTerm ) {
					r = true;
					Logger.incDepth();
					for ( Term e : t.asSet() ) {
						if ( !this.check(subType, e) ) {
							r = false;
							break;
						}
					}
					Logger.decDepth();
				} else {
					r = false;
				}
			} else if ( typeClass.equals(Term.sym("list-of")) ) {
				Term subType = type.get(1);
				if ( t instanceof ListTerm ) {
					r = true;
					Logger.incDepth();
					for ( Term e : t.asSet() ) {
						if ( !this.check(subType, e) ) {
							r = false;
							break;
						}
					}
					Logger.decDepth();
				} else {
					r = false;
				}
			} else if ( typeClass.equals(Term.sym("collection-of")) ) {
				Term subType = type.get(1);
				if ( t instanceof CollectionTerm ) {
					r = true;
					Logger.incDepth();
					for ( Term e : t.asSet() ) {
						if ( !this.check(subType, e) ) {
							r = false;
							break;
						}
					}
					Logger.decDepth();
				} else {
					r = false;
				}
			} else if ( typeClass.equals(Term.sym("signed-tuple")) ) {
				Term signature = type.get(1);
				NumericalTerm min = type.getOrDefault(Term.sym("min"), Term.integer(signature.size())).asNum();
				NumericalTerm max = type.getOrDefault(Term.sym("max"), Term.integer(signature.size())).asNum();
				NumericalTerm repeat = type.getOrDefault(Term.sym("repeat"), Term.integer(1)).asNum();
				NumericalTerm tSize = Term.integer(type.size());
				int repeat_start_idx = signature.size() - repeat.getIntValue();
				
				if ( t instanceof TupleTerm && tSize.greaterThanEq(min) && tSize.lessThanEq(max) ) {
					Logger.incDepth();
					r = true;
					for ( int i = 0 ; i < t.size() ; i++ ) {
						int sig_idx = i;
						if ( i >= signature.size() ) {
							sig_idx = repeat_start_idx + (i - signature.size()) % repeat.getIntValue();
						}
						if ( !this.check(signature.get(sig_idx), t.get(i)) ) {
							r = false;
							break;
						}
					}
					Logger.decDepth();
				} else {
					r = false;
				}
			} else if ( typeClass.equals(Term.sym("key-value-tuple")) ) {
				CollectionTerm keyTypeCol = type.get(1).asCollection();
				r = true;
				Logger.incDepth();
				for ( Term kvp : keyTypeCol ) {
					Term e = t.get(kvp.getKey());
					if ( e == null ) {
						r = false;
						break;
					}
					if ( !this.check(kvp.getValue(), e) ) {
						r = false;
						break;
					}
				}
				Logger.decDepth();
			} else if ( typeClass.equals(Term.sym("enum")) ) {
				r = type.get(1).asCollection().contains(t);
			} else if ( typeClass.equals(Term.sym("numerical-range")) ) {
				NumericalTerm min = type.getOrDefault(Term.sym("min"), Term.infNeg()).asNum();
				NumericalTerm max = type.getOrDefault(Term.sym("max"), Term.infPos()).asNum();
				r = false;
				if ( t instanceof NumericalTerm ) {
					if ( t.asNum().greaterThanEq(min) && t.asNum().lessThanEq(max) ) {
						r = true;
					}
				}
			} else if ( typeClass.equals(Term.sym("typed-key-value")) ) {
				r = false;
				if ( t instanceof KeyValueTerm ) {
					if ( this.check(type.get(1), t.getKey()) && this.check(type.get(1), t.getValue()) ) {
						r = true;
					}
				}
			} else if ( typeClass.equals(Term.sym("or-type")) ) {
				r = false;
				for ( Term choice : type.get(1).asCollection() ) {
					if ( this.check(choice, t)) {
						r = true;
						break;
					}
				}
			} else {
				throw new IllegalArgumentException("#type expression not supported: " + type);
			}
		} else if ( type instanceof SymbolicTerm ) {
			Term e = Term.tuple(type, t);
			r = this.eval.apply(e).getBooleanValue();
		} else {
			r = false;
			throw new IllegalArgumentException("#type definitions must be tuples.");
		}
		
		if ( !r ) {
			Logger.msg("TypeCheck", t + " !! " + type);
		}
		return r;
	}

}
