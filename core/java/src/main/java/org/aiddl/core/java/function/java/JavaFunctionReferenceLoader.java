package org.aiddl.core.java.function.java;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Map;

import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.representation.FunctionReferenceTerm;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class JavaFunctionReferenceLoader implements Function, ConfigurableFunction {

	private static SymbolicTerm JavaClass = Term.sym("class");
	private static SymbolicTerm InitKey = Term.sym("init");
	private static SymbolicTerm ConfigKey = Term.sym("config");
	private static SymbolicTerm Module = Term.sym("module");
	private static SymbolicTerm Name = Term.sym("name");

	FunctionRegistry fReg;
	
	@Override
	public void configure(Map<Term, Term> settings, FunctionRegistry fReg) {
		this.fReg = fReg;
	}
	
	@Override
	public Term apply(Term args) {
		Term javaClass = args.get(JavaClass);
		Term init = args.get(InitKey);
		Term config = args.get(ConfigKey);
		
		SymbolicTerm name = args.get(Name).asSym();
		SymbolicTerm module = args.get(Module).asSym();
		
		String class_name = javaClass.toString();
		
		try {
			Class<?> serviceClass = Class.forName(class_name);
			@SuppressWarnings("unchecked")
			Constructor<Function> c = (Constructor<Function>)serviceClass.getConstructor();
			Function f = c.newInstance();
			
			if ( config != null && (f instanceof ConfigurableFunction)) {
				((ConfigurableFunction)f).configure(config.asCollection().getMap(), this.fReg);
			}
			if ( init != null && (f instanceof InitializableFunction)) {
				((InitializableFunction)f).initialize(init);
			}

//			if ( name != null ) {
				SymbolicTerm uri = name;
				if ( module != null ) {
					uri = name.asSym().concat(module.asSym());
				}
				this.fReg.addFunction(uri, f);
//			}
			
//			Term fName = Term.tuple(module, name);
			FunctionReferenceTerm fName = Term.fref(uri, fReg);
			
//			this.fReg.addFunction(fName, f);
			return fName;
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
		return null;
	}
}
