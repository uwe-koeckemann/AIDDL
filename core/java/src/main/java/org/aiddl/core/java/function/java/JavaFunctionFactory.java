package org.aiddl.core.java.function.java;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;

import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.interfaces.ConfigurableFunction;
import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.interfaces.InitializableFunction;
import org.aiddl.core.java.representation.SymbolicTerm;
import org.aiddl.core.java.representation.Term;

public class JavaFunctionFactory implements Function {

	Constructor<Function> constructor;
	FunctionRegistry fReg;
		
	public JavaFunctionFactory( Constructor<Function> constructor, FunctionRegistry fReg ) {
		this.constructor = constructor;
		this.fReg = fReg;
	}

	@Override
	public Term apply(Term args) {
		SymbolicTerm uri;
		Term init = null;
		Term config = null;
		
		if ( args instanceof SymbolicTerm ) {
			uri = args.asSym();
		} else {
			uri = args.get(0).asSym();
			init = args.get(Term.sym("init"));
			config = args.get(Term.sym("config"));
		}		
		
		try {
			Function f = this.constructor.newInstance();
			
			if ( config != null && (f instanceof ConfigurableFunction)) {
				((ConfigurableFunction)f).configure(config.asCollection().getMap(), this.fReg);
			}
			if ( init != null && (f instanceof InitializableFunction)) {
				((InitializableFunction)f).initialize(init);
			}

			this.fReg.addFunction(uri, f);
			//FunctionReferenceTerm fref = Term.fref(uri, fReg);			
			return uri;
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
		} 
		return null;
	}
	
}
