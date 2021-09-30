package org.aiddl.core.function.type;

import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.FunctionReferenceTerm;
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

			} else if ( typeClass.equals(Term.sym("org.aiddl.type.set-of")) ) {
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
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.list-of")) ) {
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
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.collection-of")) ) {
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
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.tuple.signed")) ) {

				
				if ( t instanceof TupleTerm ) {
					Term signature = type.get(1);
					NumericalTerm min = type.getOrDefault(Term.sym("min"), Term.integer(signature.size())).asNum();
					NumericalTerm max = type.getOrDefault(Term.sym("max"), Term.integer(signature.size())).asNum();
					NumericalTerm repeat = type.getOrDefault(Term.sym("repeat"), Term.integer(1)).asNum();
					
					int repeat_start_idx = signature.size() - repeat.getIntValue();
					NumericalTerm tSize = Term.integer(t.size());
					if ( tSize.greaterThanEq(min) && tSize.lessThanEq(max) ) {
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
				} else {
					r = false;
				}
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.tuple.key-value")) ) {
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
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.matrix")) ) {
				if ( ((t instanceof  TupleTerm) || (t instanceof ListTerm)) && t.size() > 0 ) {
					Term colTypes = type.get(Term.sym("col-types"));
					Term rowTypes = type.get(Term.sym("row-types"));
					Term cellType = type.get(Term.sym("cell-type"));

					int m = type.getOrDefault(Term.sym("m"), Term.integer(t.size())).asInt().getIntValue();
					int n = type.getOrDefault(Term.sym("n"), Term.integer(t.get(0).size())).asInt().getIntValue();
					r = true;

					if ( t.size() != m ) r = false;
					else {
						for ( int i = 0 ; i < m ; i++ ) {
							if ( t.get(i).size() != n ) {
								r = false;
								break;
							}
							for ( int j = 0 ; j < n ; j++ ) {
								boolean cellOkay = (cellType == null) || check(cellType, t.get(i).get(j));
								boolean rowOkay = (rowTypes == null) || check(rowTypes.get(i), t.get(i).get(j));
								boolean colOkay = (colTypes == null) || check(colTypes.get(j), t.get(i).get(j));
								if ( !(cellOkay && rowOkay && colOkay) ) {
									r = false;
									break;
								}
							}
							if ( !r ) break;
						}
					}
				} else {
					r = false;
				}
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.enum")) ) {
				r = type.get(1).asCollection().contains(t);
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.range")) ) {
				NumericalTerm min = type.getOrDefault(Term.sym("min"), Term.infNeg()).asNum();
				NumericalTerm max = type.getOrDefault(Term.sym("max"), Term.infPos()).asNum();
				r = false;
				if ( t instanceof NumericalTerm ) {
					if ( t.asNum().greaterThanEq(min) && t.asNum().lessThanEq(max) ) {
						r = true;
					}
				}
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.typed-key-value")) ) {
				r = false;
				if ( t instanceof KeyValueTerm ) {
					if ( this.check(type.get(1).getKey(), t.getKey()) && this.check(type.get(1).getValue(), t.getValue()) ) {
						r = true;
					}
				}
			} else if ( typeClass.equals(Term.sym("org.aiddl.type.union")) ) {
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
		} else if ( type instanceof FunctionReferenceTerm ) {
			r = type.asFunRef().getFunction().apply(t).getBooleanValue();
		} else {
			r = false;
			throw new IllegalArgumentException("#type definition must be tuple, symbolic, or function reference. Found "+type.getClass().getSimpleName()+": " + type);
		}
		
		if ( r && (type instanceof TupleTerm) ) {
			Term constraint = type.get(Term.sym("constraint"));
			if ( constraint != null && constraint instanceof FunctionReferenceTerm ) {
				r = constraint.asFunRef().getFunction().apply(t).getBooleanValue();
				if ( !r ) {
					if ( eval.getVerbosity() >= 1  ) {
						Logger.incDepth();
						Logger.msg("TypeCheck", t + " does not satisfy " + constraint);
						Logger.decDepth();
					}
				}
			}
		}
		
		if ( !r ) {
			if ( eval.getVerbosity() >= 1  ) {
				Logger.msg("TypeCheck", t + " !! " + type);
			}
		}
		return r;
	}

}
