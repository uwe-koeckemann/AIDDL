package org.aiddl.core.function;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;

import org.aiddl.core.container.Container;
import org.aiddl.core.container.Entry;
import org.aiddl.core.eval.Evaluator;
import org.aiddl.core.function.type.GenericTypeChecker;
import org.aiddl.core.function.type.TypeChecker;
import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.interfaces.InterfaceImplementation;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.CollectionTerm;
import org.aiddl.core.representation.ListTerm;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.LockableList;
import org.aiddl.core.tools.Logger;
import org.aiddl.core.tools.TermComparator;

public class FunctionRegistry {
	private static SymbolicTerm InterfaceUri = Term.sym("uri");
	private static SymbolicTerm InterfaceInput = Term.sym("input");
	private static SymbolicTerm InterfaceOutput = Term.sym("output");
	
	private static SymbolicTerm ServiceName = Term.sym("name");
	private static SymbolicTerm ServiceClass = Term.sym("service");
	private static SymbolicTerm ConfigKey = Term.sym("config");
	
	private Map<Term, Function> functions  = new HashMap<Term,Function>();
	private Map<SymbolicTerm,List<SymbolicTerm>> implementations = new HashMap<>();
	private Map<SymbolicTerm,Term> interfaceDefinition = new HashMap<>();

	private boolean verbose = false;

	/**
	 * Create a new function registry.
	 */
	public FunctionRegistry() {
	}

	public void setVerbose( boolean verbose ) { this.verbose = verbose; }

	/**
	 * Get function registered under given name or create an anonymous function if name has form (#lambda x f).
	 * @param name name of function
	 * @return registered function or <code>null</code> if it does not exist
	 */
	public Function getFunction( Term name ) {
		if ( (name instanceof TupleTerm) &&  name.size() == 3 && name.get(0).equals(Uri.LAMBDA) ) {
			return this.lambdaFactory(name);
		}
		return this.getFunctionInternal(name.asSym());
	}
	
	/**
	 * Get function registered under given name or create an anonymous function if name has form (#lambda x f).
	 * If not function was found or created throw an {@link IllegalArgumentException}
	 * @param name name of function
	 * @return registered function or <code>null</code> if it does not exist
	 */
	public Function getFunctionOrPanic( Term name ) {
		if ( (name instanceof TupleTerm) &&  name.size() == 3 && name.get(0).equals(Uri.LAMBDA) ) {
			return this.lambdaFactory(name);
		}
		Function f = this.getFunctionInternal(name.asSym());
		
		if ( f == null ) {
			System.err.println("Registered functions:");
			for ( Term uri : this.functions.keySet() ) {
				System.err.println(uri);
			}
			throw new IllegalArgumentException("Not a registered function or lambda expression: " + name);
		}
		return f;
	}
	
	/**
	 * Get a function with a given name, or a default function if it does not exist (#lambda x f)
	 * @param name name of function
	 * @param def default function
	 * @return function registered under name or default
	 */
	public Function getFunctionOrDefault( Term name, Function def ) {
		if ( (name instanceof TupleTerm) && name.size() == 3 && name.get(0).equals(Uri.LAMBDA) ) {
			return this.lambdaFactory(name);
		}
		Function f = this.getFunctionInternal(name);
		if ( f == null ) {
			return def;
		}
		return f;
	}	
	
	private Function getFunctionInternal( Term name ) {
		Function f = this.functions.get(name);
//		if ( f == null ) {
//			List<SymbolicTerm> impl = this.implementations.get(name);
//			if ( impl != null && impl.size() > 0 ) {
//				f = this.getFunction(impl.get(0));
//			}
//		}
		return f;
	}
	
	/**
	 * Get list of all functions implementing interface 
	 * @param interfaceUri
	 * @return list term containing function URIs that implement interface
	 */
	public ListTerm getInterfaceImplementations( SymbolicTerm interfaceUri ) {
		List<SymbolicTerm> r = this.implementations.get(interfaceUri);
		if ( r == null ) {
			return Term.list();
		}
		return Term.list(r);
	}
	
	/**
	 * Directly add a function under a given name
	 */
	public void addFunction( Term name, Function f ) {
		if ( this.hasFunction(name) ) {
			this.removeInterfaceImplementation(name.asSym());
		}
		this.functions.put(name, f);
		if ( f instanceof InterfaceImplementation ) {
			SymbolicTerm interfaceUri = ((InterfaceImplementation)f).getInterfaceUri();
			this.implementations.putIfAbsent(interfaceUri, new ArrayList<>());
			this.implementations.get(interfaceUri).add(name.asSym());
		}
	}
	
	/**
	 * Directly add a function under a given name if it does not exist
	 */
	public void addFunctionIfAbsent( Term name, Function f ) {
		if ( this.functions.putIfAbsent(name, f) == null ) {
			if ( f instanceof InterfaceImplementation ) {
				SymbolicTerm interfaceUri = ((InterfaceImplementation)f).getInterfaceUri();
				this.implementations.putIfAbsent(interfaceUri, new ArrayList<>());
				this.implementations.get(interfaceUri).add(name.asSym());
			}
		}
	}
	
	public void removeFunction( Term name ) {
		this.functions.remove(name);
		this.removeInterfaceImplementation(name.asSym());
	}
	
	private void removeInterfaceImplementation( SymbolicTerm name ) {
		for ( Term k : this.implementations.keySet() ) {
			this.implementations.get(k).remove(k);
		}
	}
	
	private Function lambdaFactory( Term lambda ) {
		Term x = lambda.get(1);
		Term f = lambda.get(2);
		Evaluator e = (Evaluator)this.getFunction(Uri.EVAL);
		return new Lambda(x, f ,e);
	}
	
	private Function functionFactory( SymbolicTerm uri, Term args,  Term body, Container C ) {
		Evaluator e = (Evaluator)this.getFunction(Uri.EVAL);
		return new NamedFunction(uri, args, body ,e);
	}
	
	public void loadContainerDefintions( Container C ) {
		for ( Term m : C.getModuleNames() ) { 
			for ( Entry e : C.getMatchingEntries(m, Term.sym("#def"), Term.anonymousVar()) ) {
				if ( e.getName() instanceof SymbolicTerm ) {
					SymbolicTerm uri = m.asSym().concat(e.getName().asSym());
					Function f = this.functionFactory(uri, null, e.getValue(), C);
					this.addFunction(uri, f);
				} else if ( e.getName() instanceof TupleTerm && e.getName().size() > 0 ) {
					SymbolicTerm uri = m.asSym().concat(e.getName().get(0).asSym());
					Term args;
					if ( e.getName().size() == 2 ) {
						args = e.getName().get(1);
					} else {
						LockableList argList = new LockableList();
						for ( int i = 1 ; i < e.getName().size() ; i++ ) {
							argList.add( e.getName().get(i) );
						}
						args = Term.tuple(argList);
					}
					Function f = functionFactory(uri, args, e.getValue(), C);
					this.addFunction(uri, f);
				}
			}
		}
	}
	
	/**
	 * Create and configure functions by dynamically creating instances of Java classes.
	 * @param serviceConfigs AIDDL configuration of services
	 */
	public void loadJavaFunctions( CollectionTerm serviceConfigs ) {
		for ( Term service_config : serviceConfigs ) {
			this.loadJavaFunction(service_config);
		}
	}
	
	/**
	 * Create and configure a function by dynamically creating an instance of a Java class .
	 * @param serviceConfig
	 */
	public void loadJavaFunction( Term serviceConfig ) {
		Term name = serviceConfig.get(ServiceName);
		Term service = serviceConfig.get(ServiceClass);
		Term config = serviceConfig.get(ConfigKey);
		
		String class_name = service.toString();
		
		try {
			Class<?> serviceClass = Class.forName(class_name);
			@SuppressWarnings("unchecked")
			Constructor<Function> c = (Constructor<Function>)serviceClass.getConstructor();
			Function f = c.newInstance();
			
			if ( config != null && (f instanceof ConfigurableFunction)) {
				((ConfigurableFunction)f).configure(config.asCollection().getMap(), this);
			}
			
			if ( f instanceof InterfaceImplementation ) {
				SymbolicTerm interfaceDefinition = ((InterfaceImplementation)f).getInterfaceUri();
//				this.addInterface()
			}
			
			this.functions.put(name, f);
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (SecurityException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (InstantiationException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (IllegalAccessException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (InvocationTargetException e) {
			e.getTargetException().printStackTrace();
			e.printStackTrace();
			System.exit(1);
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (NoSuchMethodException e) {
			e.printStackTrace();
			System.exit(1);
		}
	}
	
	public Collection<Term> getRegisteredNames() {
		return functions.keySet();
	}
	
	public Collection<Term> getRegisteredNamesSorted() {
		List<Term> names = new ArrayList<>();
		names.addAll(this.functions.keySet());
		Collections.sort(names, new TermComparator());
		return names;
	}
	
	public boolean hasFunction( Term fName ) {
		return this.functions.containsKey(fName);
	}
		
	public void loadContainerInterfaces( Container C ) {
		Evaluator eval = (Evaluator) this.getFunction(Uri.EVAL);
		for ( Term m : C.getModuleNames() ) { 
			for ( Entry e : C.getMatchingEntries(m, Term.sym("#interface"), Term.anonymousVar()) ) {
				SymbolicTerm uri = eval.apply(e.getValue().get(InterfaceUri)).asSym();
				this.interfaceDefinition.put(uri, eval.apply(e.getValue()));
				
			}
		}
	}
	
	public void loadTypeFunctions( Container C ) {
		Evaluator eval = (Evaluator)this.getFunction(Uri.EVAL);
		
		for ( int j = C.getModuleNames().size()-1 ; j >= 0; j-- ) {
			Term m = C.getModuleNames().get(j);
			if ( verbose ) {
				Logger.msg("FunReg", "Looking for types in module: " + m);
				Logger.incDepth();
			}

			Collection<Entry> all = C.getMatchingEntries(m, Term.sym("#type"), Term.anonymousVar());
			for ( Entry e : all ) {
				Term name = e.getName();
				if ( name instanceof SymbolicTerm ) {
					Term typeUri = m.asSym().concat(name.asSym());

					eval.setEvalAllReferences(true);
					Term typeDef = eval.apply(e.getValue());
					eval.setEvalAllReferences(false);
					Function typeFun = new TypeChecker(typeDef, eval);
					if ( verbose ) Logger.msg("FunReg", "Registered type: " + typeUri);
					this.addFunction(typeUri, typeFun);

				} else if ( name instanceof TupleTerm ) {
					SymbolicTerm baseUri = m.asSym().concat(name.get(0).asSym());

					eval.setEvalAllReferences(true);
					Term typeDef = eval.apply(e.getValue());
					eval.setEvalAllReferences(false);
					LockableList genArgsList = new LockableList();
					for ( int i = 1 ; i < name.size() ; i++ ) {
						genArgsList.add(name.get(i));
					}
					Term genArgs = genArgsList.size() == 1 ? genArgsList.get(0) : Term.tuple(genArgsList);

					Function genericTypeChecker = new GenericTypeChecker(baseUri, genArgs, typeDef, eval, this);
					if ( verbose ) Logger.msg("FunReg", "Registered generic type: " + baseUri);
					this.addFunction(baseUri, genericTypeChecker);
				} else {
					throw new IllegalArgumentException("#type entry name not symbolic or tuple: " + name);
				}
			}
			if ( verbose ) Logger.decDepth();
		}
	}

	public Term getInterfaceDefinition( SymbolicTerm uri ) {
		return this.interfaceDefinition.get(uri);
	}
	
	public Function getInputChecker( SymbolicTerm uri ) {
		if ( !this.interfaceDefinition.containsKey(uri) ) {
			List<Term> names = new ArrayList<>();
			names.addAll(this.interfaceDefinition.keySet());
			Collections.sort(names, new TermComparator());
			System.err.println("=======================");
			System.err.println("= Known interfaces:");
			System.err.println("=======================");
			for ( Term name : names ) {
				System.err.println(name);
			}
			
			
			throw new IllegalArgumentException("Unknown interface: " + uri);
		}
		return this.interfaceDefinition.get(uri).get(InterfaceInput).asFunRef().getFunction();
	}
	
	public Function getOutputChecker( SymbolicTerm uri ) {
		if ( !this.interfaceDefinition.containsKey(uri) ) {
			List<Term> names = new ArrayList<>();
			names.addAll(this.interfaceDefinition.keySet());
			Collections.sort(names, new TermComparator());
			System.err.println("=======================");
			System.err.println("= Known interfaces:");
			System.err.println("=======================");
			for ( Term name : names ) {
				System.err.println(name);
			}
			
			
			throw new IllegalArgumentException("Unknown interface: " + uri);
		}
		return this.interfaceDefinition.get(uri).get(InterfaceOutput).asFunRef().getFunction();
	}
}
