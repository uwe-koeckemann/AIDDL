package org.aiddl.core.function;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;

import org.aiddl.core.container.Container;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.KeyValueTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.NumericalTerm;
import org.aiddl.core.representation.ReferenceTerm;
import org.aiddl.core.representation.SetTerm;
import org.aiddl.core.representation.StringTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.representation.VariableTerm;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.LockableList.ListType;
import org.aiddl.core.tools.LockableSet;
import org.aiddl.core.tools.MapType;
import org.aiddl.core.tools.Logger;

/**
 * Main class to evaluate expressions. Each concrete evaluator implements {@link Function}.
 * The eval function will evaluate all parts of an expression recursively before
 * evaluating the expression itself. This can be avoided by marking the terms
 * representing expressions for delayed evaluation. Delayed evaluation is useful
 * when a result may become known after evaluating arguments only partially
 * (e.g., logical and becomes false as soon as one of its arguments evaluates
 * to false.  
 * 
 * @author Uwe Köckemann
 *
 */
public class Evaluator implements Function, ConfigurableFunction {
	
	private Container db; 
	private FunctionRegistry fReg;
	
	private Function identity = (Term x)->{return x;};
	
	private static final Term SELF = Term.sym("#self");
	private static final Term SELF_ALT = Term.sym("#arg");
	
	private Set<Term> delayedEval = new HashSet<>();
	
	private Stack<Substitution> selfStack = new Stack<>();  
	
	private int verbosity = 0;
	
	private int max_recursion_depth = 0;
	private int recursion_depth = 0;
	private Map<Integer,Integer> call_counter = new HashMap<>();
	
	private boolean follow_reference = false;

	/**
	 * Create a new evaluator. 
	 * @param db container used to resolve references
	 */
	public Evaluator( FunctionRegistry fReg, Container db ) {
		this.fReg = fReg;
		this.db = db;
		
		delayedEval.add(DefaultFunctions.LAMBDA);
		delayedEval.add(DefaultFunctions.QUOTE);
		delayedEval.add(DefaultFunctions.FORALL);
		delayedEval.add(DefaultFunctions.EXISTS);
		delayedEval.add(DefaultFunctions.MATCH);
		delayedEval.add(DefaultFunctions.IF);
		delayedEval.add(DefaultFunctions.COND);
		delayedEval.add(DefaultFunctions.ZIP);
		delayedEval.add(DefaultFunctions.MAP);
		delayedEval.add(DefaultFunctions.FILTER);
		delayedEval.add(DefaultFunctions.REDUCE);
		delayedEval.add(DefaultFunctions.AND);
		delayedEval.add(DefaultFunctions.OR);
		delayedEval.add(DefaultFunctions.EVAL_REF);
		delayedEval.add(DefaultFunctions.DOMAIN);
		delayedEval.add(DefaultFunctions.LET);
		delayedEval.add(DefaultFunctions.EVAL_ALL_REFS);
	}
		
	/**
	 * Set this evaluator's verbose flag. 
	 * @param flag
	 */
	public void setVerbose ( int verbosity ) {
		this.verbosity = verbosity;
	}
	
	/**
	 * Get verbosity setting
	 * @return verbosity level
	 */
	public int getVerbosity() {
		return this.verbosity;
	}
	
	/**
	 * Set this evaluator's follow references flag. 
	 * @param flag
	 */
	public void setEvalAllReferences( boolean flag ) {
		this.follow_reference = flag;
	}
	
	/**
	 * Set the container that is used to resolve
	 * references. 
	 * @param db
	 */
	public void setContainer( Container db ) {
		this.db = db;
	}
	
	Map<Term, Term> cache = new HashMap<>();
	
	@Override
	public void configure( Map<Term, Term> settings, FunctionRegistry fReg ) {
		this.fReg = fReg;
	}
	
	
	private Map<Term, Boolean> evaluatableCache = new HashMap<>();
	
	private boolean evaluatable( Term arg ) {
		Boolean answer = null;
		if ( arg instanceof SymbolicTerm )
			answer = false;
		else if ( arg instanceof NumericalTerm )
			answer = false;		
		else if ( arg instanceof VariableTerm )
			answer = false;
		else if ( arg instanceof StringTerm )
			answer = false;	
		else if ( arg instanceof ReferenceTerm ) {
			answer = true; // evaluatable(this.db.resolveReference(arg.asRef()));
		} else if ( arg instanceof FunctionReferenceTerm ) {
			answer = false; // evaluatable(this.db.resolveReference(arg.asRef()));
		}
		else if ( arg instanceof KeyValueTerm ) {
			answer = evaluatable(arg.getKey()) || evaluatable(arg.getValue());
		}
		else if ( arg instanceof CollectionTerm ) {
			for ( Term c : arg.asCollection() ) {
				if ( evaluatable(c) ) {
					answer = true;
					break;
				}
			}
			if ( answer == null )
				answer = false;
		}
		else if ( arg instanceof TupleTerm ) {
			if ( arg.size() == 0 ) 
				answer = false;
			else if ( fReg.hasFunction(arg.get(0)) ) {
				answer = true;
			}
			if ( answer == null ) {
				for ( int i = 0 ; i < arg.size() ; i++ ) {
					if ( evaluatable(arg.get(i)) ) {
						answer = true;
						break;
					}
				}
			}
			if ( answer == null ) 
				answer = false;
		}
		if ( answer == null ) {
			answer = false;
		}
//		evaluatableCache.put(arg, answer);
		return answer;
	}
	
	public void resetCallCounter() {
		this.recursion_depth = 0;
		this.call_counter.clear();
		this.max_recursion_depth = 0;
	}
	
	public List<Integer> getCallCounter() {
		List<Integer> r = new ArrayList<Integer>();
		for ( int i = 1; i < max_recursion_depth; i++ ) {
			r.add(call_counter.get(i));
		}
		return r;
	}
	
	public Term apply( Function f, Term arg ) {
		return f.apply(this.apply(arg));
	}
		
	/**
	 * Evaluate a term based on a set of registered evaluation functions.
	 * @param arg term we want to evaluate
	 * @return the result if a known function was provided, <code>arg</code> otherwise
	 */
	@Override
	public Term apply( Term arg ) {		
		Term result = null;

		if ( !this.evaluatable(arg) ) {
			return arg;
		}
		
		recursion_depth++;
		
		call_counter.putIfAbsent(recursion_depth, 0);
		Integer current = call_counter.get(recursion_depth);
		call_counter.put(recursion_depth, current+1);
		if ( recursion_depth > max_recursion_depth ) {
			max_recursion_depth = recursion_depth;
		}

		if ( verbosity == 1 ) {
			Logger.msg("EVAL", arg.toString());
			Logger.incDepth();
		}
		
		String tail = "";
		
		if ( !(arg instanceof TupleTerm) || arg.size() == 0 ) {
			if ( arg instanceof ListTerm ) {
				if ( verbosity >= 2 ) {
					Logger.msg("EVAL", "List: " + arg.toString().substring(0, Math.min(30, arg.toString().length())));
					Logger.incDepth();
					Logger.msg("EVAL", "Argument", arg.toString());	
					Logger.msg("EVAL", "Recursive Eval");
					Logger.incDepth();
				} 
										
				LockableList newCol = new LockableList();
				for ( Term t : arg.asList() ) {
					newCol.add(this.apply(t));
				}
				result = Term.list(newCol);
			} else if ( arg instanceof SetTerm ) {
				if ( verbosity >= 2 ) {
					Logger.msg("EVAL", "Set: " + arg.toString().substring(0, Math.min(30, arg.toString().length())));
					Logger.incDepth();
					Logger.msg("EVAL", "Argument", arg.toString());	
					Logger.msg("EVAL", "Recursive Eval");
					Logger.incDepth();
				} 
										
				LockableSet newCol = new LockableSet();
				for ( Term t : arg.asSet() ) {
					newCol.add(this.apply(t));
				}
				result = Term.set(newCol);
			} else if ( arg instanceof KeyValueTerm ) {
				if ( verbosity >= 2 ) {
					Logger.msg("EVAL", "Key Value Term: " + arg.toString().substring(0, Math.min(30, arg.toString().length())));
					Logger.incDepth();
					Logger.msg("EVAL", "Argument", arg.toString());
					Logger.msg("EVAL", "Recursive Eval");
					Logger.incDepth();
				}
				result = Term.keyVal(this.apply(arg.getKey()), this.apply(arg.getValue()));
			} else if ( arg instanceof ReferenceTerm ) {
				if ( verbosity >= 2 ) {
					Logger.msg("EVAL", "Reference: " + arg.toString().substring(0,  Math.min(30, arg.toString().length())));
					Logger.incDepth();
					Logger.msg("EVAL", "Argument", arg.toString());
					Logger.msg("EVAL", "Recursive Eval");
					Logger.incDepth();
				}
				if ( this.follow_reference ) {
					result = this.apply(db.resolveReference((ReferenceTerm)arg));
				} else {
					result = db.resolveReference((ReferenceTerm)arg);
				}
			} else {
				if ( verbosity >= 2 ) {
					Logger.msg("EVAL", "Function Reference: " + arg.toString());
					Logger.incDepth();
					Logger.incDepth();
				}
				result = arg;
			}
		} else {
			boolean doPop = false;
			Term operator = arg.get(0);	
		
			if ( operator instanceof ReferenceTerm ) {
				if ( operator.asRef().getRefTarget() instanceof SymbolicTerm ) {
					SymbolicTerm uri = operator.asRef().convert2uri();
					if ( fReg.getFunction(uri) != null ) {
						operator = uri;
					} else {
						operator = db.resolveReference((ReferenceTerm) operator);
					}
				} else {
					operator = db.resolveReference((ReferenceTerm) operator);
				}
			}

			if ( verbosity > 0 ) {
				tail = operator.toString();
				if ( verbosity >= 2 ) {
					Logger.msg("EVAL", "Function: " + arg.toString().substring(0, Math.min(30, arg.toString().length())));
					Logger.incDepth();
					Logger.msg("EVAL", "Argument", Logger.prettyPrint(arg, 0));
					Logger.msg("EVAL", "Recursive Eval");
					Logger.incDepth();
				}
			}
			
			
			LockableList resolvedArguments = new LockableList(ListType.ArrayList, MapType.LinkedHashMap);
			
			Term processed_op = this.apply(operator);
			if ( !fReg.hasFunction(processed_op) ) {
				resolvedArguments.add(processed_op);	
			}
			
			for ( int i = 1 ; i < arg.size() ; i++ ) {
				if ( !delayedEval.contains(operator)) {
					Term res_arg;
					if ( arg.get(i) instanceof ReferenceTerm )
						res_arg = db.resolveReference((ReferenceTerm) arg.get(i));
					else
						res_arg = arg.get(i);
					
					if ( !this.selfStack.isEmpty() ) {
						if ( !operator.equals(DefaultFunctions.TYPE) ) {
							res_arg = res_arg.substitute(this.selfStack.peek());
						} else if ( i == 1 ) {
							res_arg = res_arg.substitute(this.selfStack.peek());
						}
					}
		
					if ( operator.equals(DefaultFunctions.TYPE) && i == 2 ) {
						Substitution s = new Substitution();
						s.add(SELF, resolvedArguments.get(0));
						s.add(SELF_ALT, resolvedArguments.get(0));

						res_arg = res_arg.substitute(s);
						selfStack.push(s);
						
						doPop = true;
					}
					
					res_arg = this.apply(res_arg);
					resolvedArguments.add(res_arg);
				} else {
					resolvedArguments.add(arg.get(i));
				}
			}
			Term processedArgs;
			if ( resolvedArguments.size() == 1 ) { // && resolvedArguments.get(0) instanceof TupleTerm ) {
				processedArgs = resolvedArguments.get(0); //.asTuple();
			} else {
				processedArgs = Term.tuple(resolvedArguments);
			}
			try {
				if ( verbosity >= 1 ) {
					Logger.msg("EVAL", "Calling: " + operator +"  with " + processedArgs);
				}
				result = fReg.getFunctionOrDefault(operator, identity).apply(processedArgs);
			} catch ( Exception e ) {
				System.err.println("Registry null? " + (fReg == null));
				System.err.println("Operator: " + Logger.prettyPrint(operator,1));
				System.err.println("Arguments: " + Logger.prettyPrint(processedArgs, 1));
				e.printStackTrace();
				
				System.exit(0);
			} catch ( Error e ) {
				System.err.println("Registry null? " + (fReg == null));
				System.err.println("Operator: " + Logger.prettyPrint(operator, 1));
				System.err.println("Arguments: " + Logger.prettyPrint(processedArgs, 1));
				e.printStackTrace();
				System.exit(0);
			}
			
			
			if ( doPop ) selfStack.pop();
		}
		if ( verbosity > 0 ) {
			if ( verbosity >= 2 ) {
				Logger.decDepth();
				Logger.msg("EVAL", "Result " + result.toString() + " // " + tail, result.toString());
				Logger.decDepth();
			} else {
				Logger.decDepth();
				Logger.msg("EVAL", result.toString() + " // " + tail);
			}
		}
		
		recursion_depth--;
		return result;
	}
	
	/**
	 * Add function to evaluator
	 * @param fName name of function
	 * @param f the function
	 * @param delayed if <code>true</code> recursive evaluation is taken care of in the function. Otherwise {@link Evaluator} takes care of this 
	 */
	public void register( Term fName, Function f, boolean delayed ) {
		this.fReg.addFunctionIfAbsent(fName, f);
		if ( delayed ) {
			this.delayedEval.add(fName);
		}
	}
}
