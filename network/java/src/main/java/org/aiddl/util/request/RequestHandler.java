package org.aiddl.util.request;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Scanner;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.function.DefaultFunctions;
import org.aiddl.core.function.Evaluator;
import org.aiddl.core.function.FunctionRegistry;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InitializableFunction;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.FunctionReferenceTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.ReferenceTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.ComboIterator;
import org.aiddl.core.tools.Global;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.tools.StopWatch;

/**
 * Manages and executes services and service requests
 *
 * @author Uwe Koeckemann
 */
public class RequestHandler {
	
	public final static SymbolicTerm While = Term.sym("while");
	public final static SymbolicTerm ForAll = Term.sym("forall");
	public final static SymbolicTerm Match = Term.sym("match");
	public final static SymbolicTerm If = Term.sym("if");
	public final static SymbolicTerm Loop = Term.sym("loop");
	public final static SymbolicTerm Debug = Term.sym("debug");
	private static SymbolicTerm StopWatchStart = Term.sym("start");
	private static SymbolicTerm StopWatchStop = Term.sym("stop");

	private FunctionRegistry fReg;
		
	private String name = RequestHandler.class.getSimpleName();
	private boolean verbose = false;
	private boolean enforceTypeChecking = false;
	
	private Evaluator eval;
	
	/**
	 * Create new request handler
	 * @param eval evaluator used by request handler
	 */
	public RequestHandler( FunctionRegistry fReg ) {
		this.fReg = fReg;
		this.eval = (Evaluator)fReg.getFunction(DefaultFunctions.EVAL);
	}
	
	/**
	 * Create and configure services from a collection of configuration entries.
	 * @param serviceConfigs AIDDL configuration of services
	 */
	public void loadServices( CollectionTerm serviceConfigs ) {
		fReg.loadJavaFunctions(serviceConfigs);
	}
	
	public void addFunction( Term defName, Function f ) {
		fReg.addFunction(defName, f);
	}
		
	/**
	 * Set the request handler to verbose
	 * @param flag Boolean determining if verbose mode is on (<code>true</code>) or off (<code>false</code>)
	 */
	public void setVerbose( boolean flag ) {
		this.verbose = flag;
	}
	
	/**
	 * Set the flag for the request handler to enforce type checking
	 * @param flag Boolean determining if type checking on (<code>true</code>) or off (<code>false</code>)
	 */
	public void setEnforceTypeCheck( boolean flag ) {
		this.enforceTypeChecking = flag;
	}
	
	/**
	 * 
	 * @param request
	 * @param db
	 * @param exec_module 
	 */
	public void satisfyRequest( Term request, Container db, Term exec_module ) {	
		eval.setContainer(db);
		if ( request instanceof ListTerm ) {
			if ( verbose ) {
				Logger.msg(name, "begin list" + request);
				Logger.incDepth();
			}
			for ( Term sub_request : request.asList() ) {
				satisfyRequest(sub_request, db, exec_module);
			}
			if ( verbose ) {
				Logger.decDepth();
			}
		} else if  ( request instanceof ReferenceTerm ) {		
			Term refTarget = request.getRefTarget();
			if ( refTarget instanceof ReferenceTerm ) {
				refTarget = db.resolveReference((ReferenceTerm)refTarget);
			}
			Entry resolvedReq = db.getEntry(request.getRefModule(), refTarget);

			if ( resolvedReq == null ) {
				if ( verbose ) {
					Logger.msg(name, "Entry "+request.getRefTarget() +" not found in working module. Looking for matches in all modules...");
					Logger.incDepth();
				}
				Collection<Entry> entries = db.getMatchingEntries(null, null, request);
				for ( Entry e : entries ) {
					if ( verbose ) Logger.msg(name, "Match: " + e.getValue());
					satisfyRequest(e.getValue(), db, exec_module);
				}
				if ( verbose && entries.isEmpty() ) {
					Logger.msg(name, "Could not find any entries matching: " + request);
				}
				if ( verbose ) {
					Logger.decDepth();
				}

			} else {
				if ( verbose ) {
					Logger.msg(name, "Reference: " + request + " ("+request.getRefTarget().resolve(db)+") resolved to " + resolvedReq);
					Logger.incDepth();
				}
				satisfyRequest(resolvedReq.getValue(), db, exec_module);
				
				if ( verbose ) Logger.decDepth();
			}	
		} else if ( request instanceof TupleTerm ) {
			TupleTerm reqTuple = (TupleTerm) request;
			
			if ( reqTuple.get(0).equals(Term.sym("if")) ) {
				Term condition = reqTuple.get(1).resolve(db);
				
				if ( verbose ) {
					Logger.msg(name, "if " + condition);
					Logger.msg(name, "-> " + eval.apply(condition));
				}
				if ( eval.apply(condition).getBooleanValue() ) {
					if ( verbose ) Logger.msg(name, "--> then: " + reqTuple.get(2));
					satisfyRequest(reqTuple.get(2), db, exec_module);
				} else if ( reqTuple.size() == 4 ) {
					if ( verbose ) Logger.msg(name, "--> else: " + reqTuple.get(3));
					satisfyRequest(reqTuple.get(3), db, exec_module);
				}
			} else if ( reqTuple.get(0).equals(Term.sym("write")) ) {
				Term value = eval.apply( reqTuple.get(1).resolve(db) );
				ReferenceTerm ref = out2ref(reqTuple.get(2).resolve(db), db, exec_module);
				
				if ( verbose ) {
					Logger.msg(name, "write " + value + " to " + ref);
				}
				
				Entry prev = db.getEntry(ref.getRefModule(), ref.getRefTarget());
				Term prevType;
				if ( prev == null ) {
					prevType = DefaultFunctions.TYPE_TERM; // Term.sym("#term");
				} else {
					prevType = prev.getType();
				}
				Entry outEntry = new Entry( prevType, ref.getRefTarget(), value);
				if ( verbose ) Logger.msg(name, "Writing to DB: " + outEntry.toString());
				db.setEntry(ref.getRefModule(), outEntry);				
			} else if ( reqTuple.get(0).equals(Term.sym("create")) ) {
				Term entry_term = eval.apply( reqTuple.get(1) );
				
				if ( verbose ) {
					Logger.msg(name, "creating entry " + entry_term);
				}
				
				Term type = entry_term.get(0);
				Term name = entry_term.get(1);
				Term value = entry_term.get(2);
				
				if ( enforceTypeChecking ) {
//					Function tCheck = eval.compute(type).asFunRef().getFunction(fReg);
//					System.out.println("F: " + tCheck);
					Function tCheck = type.asFunRef().getFunction();
					
					if ( !( type instanceof FunctionReferenceTerm ) ) {
						System.err.println("Request: " + request);
						System.err.println("Type:    " + type);
						System.err.println("Name:    " + name);
						System.err.println("Value:   " + value);
						throw new IllegalArgumentException("Type is not a function reference:" + type);
					}					
					if ( tCheck == null ) {
						System.err.println("Request: " + request);
						System.err.println("Type:    " + type);
						System.err.println("Name:    " + name);
						System.err.println("Value:   " + value);
						throw new IllegalArgumentException("Function not registered and not a lambda expression:\n" + type);
					}
					
					Term typeSat = tCheck.apply(value);
					if ( !typeSat.getBooleanValue() ) {
						System.err.println("Request: " + request);
						System.err.println("Type:    " + type);
						System.err.println("Name:    " + name);
						System.err.println("Value:   " + value);
						eval.setVerbose(2);
						tCheck.apply(value);
						throw new IllegalArgumentException("Value does not satisfy type and type checking is enforced in RequestHandler.");
					}
				}				
				
				Entry prev = new Entry(type, name, value);
				
				db.setEntry(exec_module, prev);				
			} else if ( reqTuple.get(0).equals(Term.sym("print")) ) {
				Term name = eval.apply( reqTuple.get(1).resolve(db) );
				Term value = eval.apply( reqTuple.get(2).resolve(db) ) ;
				
				Logger.msg(name.toString().replace("\"", ""), Logger.prettyPrint(value, Logger.depth()));
			} else if ( reqTuple.get(0).equals(Term.sym("debug")) ) {
				Logger.msg(this.name, "Breakpoint reached");
				Scanner s = new Scanner(System.in);
//				String query = null;
				System.out.print("$");
				while ( s.nextLine() != "q" ) {
					System.out.print("$");
				}			
				s.close();
			} else if ( reqTuple.get(0).equals(Term.sym("stopwatch")) ) {
				Term command = eval.apply( reqTuple.get(1) );
				Term value = eval.apply( reqTuple.get(2) );
				if ( command.equals(StopWatchStart) ) {
					StopWatch.start(value.toString());
				} else if ( command.equals(StopWatchStop) ) {
					StopWatch.stop(value.toString());
				} else {
					throw new IllegalArgumentException("Unknown stopwatch command: " + command + ". Use start or stop.");
				}
			} else if ( reqTuple.get(0).equals(Term.sym("init")) ) {
				Term name = eval.apply( reqTuple.get(1).resolve(db) );
				Term arg = eval.apply( reqTuple.get(2).resolve(db) ); 		
				if ( verbose ) {
					Logger.msg(this.name, "initializing " + name);
				}
				Function f = this.fReg.getFunction(name);
				if ( f == null ) {
					throw new IllegalArgumentException("Function not registered: " + name);
				}
				if ( !(f instanceof InitializableFunction)  ) {
					throw new IllegalArgumentException("Function " + name + " (class: " + f.getClass()+") does not implement InitializableFunction");
				}
				((InitializableFunction)f).initialize(arg);
			} else if ( reqTuple.get(0).equals(Term.sym("call")) ) {
				Term name = eval.apply( reqTuple.get(1).resolve(db) );
				Term input = reqTuple.get(2).resolve(db); 		
				Term output = reqTuple.get(3).resolve(db);
				if ( verbose ) {
					Logger.msg(this.name, "Calling: " + name + " with " + input);
					Logger.incDepth();
				}
				Function f = this.fReg.getFunction(name);
				if ( f == null ) {
					throw new IllegalArgumentException("Function not registered: " + name + "\n\tregistered: " + fReg.getRegisteredNames());
				}
					
				input = input.resolve(db);
				input = eval.apply(input);
				
				SymbolicTerm interfaceUri = null;
				
				if ( enforceTypeChecking && f instanceof InterfaceImplementation ) {
					interfaceUri = ((InterfaceImplementation)f).getInterfaceUri();
					
					Function t_checker = fReg.getInputChecker(interfaceUri);
					StopWatch.start("Type Checking");
					if ( !eval.apply(t_checker, input).getBooleanValue() ) {
						Logger.startLoggingToFile(Global.workDir() + "/type-check-failure.org", Logger.OrgTabbing);
						eval.setVerbose(2);
						eval.apply(t_checker, input).getBooleanValue();
						Logger.stopLoggingToFile(Global.workDir() + "/type-check-failure.org");
						throw new IllegalArgumentException("\nRequest: " + request + "\nwith resolved input: " + Logger.prettyPrint(input, 1) + "\ndoes not satisfy input of declaration: " + fReg.getInterfaceDefinition(interfaceUri));
					}
					StopWatch.stop("Type Checking");
					if ( verbose ) Logger.msg(this.name, "Input type check passed.");
				}
				Term result = f.apply(input);
				
				if ( verbose ) Logger.msg(this.name, "Result: " + result);
				
				if ( enforceTypeChecking && f instanceof InterfaceImplementation ) {
					interfaceUri = ((InterfaceImplementation)f).getInterfaceUri();
					Function t_checker = fReg.getOutputChecker(interfaceUri);
					StopWatch.start("Type Checking");
					if ( !eval.apply(t_checker, result).getBooleanValue() ) {
//						TypeChecker.setUseCache(false);
//						TypeChecker.setVerbose(2);
						eval.setVerbose(2);
						Logger.startLoggingToFile(Global.workDir() + "/type-check-failure.org", Logger.OrgTabbing);
						eval.apply(t_checker, result);
						Logger.stopLoggingToFile(Global.workDir() + "/type-check-failure.org");
						throw new IllegalArgumentException("\nRequest: " + request + "\nwith produced result: " + Logger.prettyPrint(result, 1) + "\ndoes not satisfy ouput of interface: " + fReg.getInterfaceDefinition(interfaceUri));
					}
					StopWatch.stop("Type Checking");
					if ( verbose ) Logger.msg(this.name, "Output type check passed.");
//					if ( !((InterfaceImplementation)f).getType().checkOutput(result, db, eval) ) {
//						TypeChecker.setUseCache(false);
//						TypeChecker.setVerbose(2);
//						Logger.startLoggingToFile("type-check-failure.org", Logger.OrgTabbing);
//						((InterfaceImplementation)f).getType().checkOutput(result, db, eval);
//						Logger.stopLoggingToFile("type-check-failure.org");
//						throw new IllegalArgumentException("\nRequest: " + request + "\nproduced result: " + Logger.prettyPrint(result, 1) + "\ndoes not satisfy output of declaration: " + ((InterfaceImplementation)f).getType());
//					}
				}
				
				if ( output instanceof CollectionTerm ) {
					
					for ( Term out : output.asCollection() ) {
						Term key = out.getKey();
						ReferenceTerm ref = out2ref(out.getValue(), db, exec_module);
						
						Term value = result.get(key);
						if ( value == null ) {
							throw new IllegalStateException("\nRequest: " + request +"\n"
														  + "returned: " + result + "\n"
														  + "missing key: " + key + "\n"
														  + "from output specification: " + output + "\n"
														  + "Make sure the keys in the output specification are the same as used in the result returned by the requested function.");
						}
						Entry prev = db.getEntry(ref.getRefModule(), ref.getRefTarget());
						Term prevType;
						if ( prev == null ) {
							prevType = Term.sym("#term");
						} else {
							prevType = prev.getType();
						}
						Entry outEntry = new Entry( prevType, ref.getRefTarget(), value);
						if ( verbose ) Logger.msg(this.name, "Writing to DB: " + outEntry.toString());
						db.setEntry(ref.getRefModule(), outEntry);
					}
				} else {
					ReferenceTerm ref = out2ref(output, db, exec_module);
					Entry prev = db.getEntry(ref.getRefModule(), ref.getRefTarget());
					Term prevType;
					if ( prev == null ) {
						prevType = Term.sym("#term");
					} else {
						prevType = prev.getType();
					}
					Entry outEntry = new Entry(prevType, ref.getRefTarget(), result);
					if ( verbose ) Logger.msg(this.name, "Writing to DB: " + outEntry.toString());
					db.setEntry(ref.getRefModule(), outEntry);
				}
				if ( verbose ) {
					Logger.decDepth();
				}
				
			}  else if ( reqTuple.get(0).equals(Term.sym("match")) ) {
				Term a = reqTuple.get(1).resolve(db);
				Term b = reqTuple.get(2).resolve(db);
				Term sub_request = reqTuple.get(3);
				Substitution s = a.match(b);
				
				if ( verbose ) Logger.msg(name, String.format("Matching %s to %s -> %s", a.toString(), b.toString(), s));
				
				if ( s != null ) {
					this.satisfyRequest(sub_request.substitute(s), db, exec_module);
				}
				
			} else if ( reqTuple.get(0).equals(Term.sym("forall")) ) {
				ListTerm domainMap = (ListTerm) reqTuple.get(1).resolve(db);
				
				Term sub_request = reqTuple.get(2);
				List<Term> variables = new ArrayList<Term>();
				
				List<List<Term>> choices = new ArrayList<List<Term>>();
				for ( Term var_choice : domainMap ) {
					variables.add(var_choice.getKey());
					CollectionTerm domain = eval.apply(var_choice.getValue()).asCollection();
					List<Term> domainList = new ArrayList<>();
					domain.addAllTo(domainList);
					choices.add( domainList );
				}
				
				if ( verbose ) Logger.msg(name, "forall choices: " + choices);
				
				ComboIterator<Term> combos = new ComboIterator<>(choices);
				for ( List<Term> combo : combos ) {
					Substitution s = new Substitution();
					for ( int i = 0 ; i < combo.size() ; i++ ) {
						s.add(variables.get(i), combo.get(i) );
					}
					Term sub_request_instance = sub_request.substitute(s);
					if ( verbose ) {
						Logger.msg(name, "Calling: " +  sub_request_instance);
						Logger.incDepth();
					}
					satisfyRequest(sub_request_instance, db, exec_module);
					if ( verbose ) Logger.decDepth();
				}
			} else if ( reqTuple.get(0).equals(Term.sym("while")) ) {
				if ( verbose ) {
					Logger.msg(name, "begin while");
					Logger.incDepth();
				}
				Term condition = reqTuple.get(1).resolve(db);
				
				while ( eval.apply(condition).getBooleanValue() ) {
					if ( verbose ) Logger.msg(name, "while condition satisfied: " + condition);
					satisfyRequest(reqTuple.get(2), db, exec_module);
					condition = reqTuple.get(1).resolve(db);
				}
				
				if ( verbose ) {
					Logger.decDepth();
				}
			} else if ( reqTuple.get(0).equals(Term.sym("loop")) ) {
				while ( true ) {
					if ( verbose ) {
						Logger.msg(name, "Loop starts...");
						Logger.incDepth();
					}
					satisfyRequest(reqTuple.get(1), db, exec_module);
					if ( verbose ) Logger.decDepth();
				}				
			}  else {
				throw new IllegalArgumentException("Cannot satisfy request: " + request);
			}
		}
	}
		
	public void satisfyRequest( Entry request, Container db, Term exec_module ) {
//		if ( enforceTypeChecking ) 
//			TypeChecker.checkAndThrow(request, db, this.eval);
		satisfyRequest(request.getValue(), db, exec_module);
	}

	private ReferenceTerm out2ref( Term out, Container db, Term exec_module ) {
		if ( out instanceof SymbolicTerm ) {
			return Term.ref(out, exec_module );
		} else if ( out instanceof TupleTerm && out.size() == 2 ) {
			return Term.ref(out.get(0), out.get(1).resolve(db));
		}
		throw new IllegalArgumentException("Bad format: " + out + "\nTerm used to describe function output should either be symbolic (target in working module) or tuple of size two (reference target and module resp.).");  
	}
}
