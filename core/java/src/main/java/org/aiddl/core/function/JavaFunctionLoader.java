package org.aiddl.core.function;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Map;

import org.aiddl.core.interfaces.ConfigurableFunction;
import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

/**
 * Function that takes name, module, java class, and configuration and adds a function to
 * its registry. Returns name of registered function.
 * 
 * @author Uwe Köckemann
 *
 */
public class JavaFunctionLoader implements Function {
	
	private static SymbolicTerm FunctionName = Term.sym("name");
	private static SymbolicTerm ModuleName = Term.sym("module");
	private static SymbolicTerm JavaClass = Term.sym("class");
	private static SymbolicTerm ConfigKey = Term.sym("config");

	FunctionRegistry fReg;
	
	public JavaFunctionLoader(FunctionRegistry fReg) {
		this.fReg = fReg;
	}
	
	@Override
	public Term apply(Term args) {
		Term name = args.get(FunctionName);
		Term module = args.get(ModuleName);		
		Term javaClass = args.get(JavaClass);
		Term config = args.get(ConfigKey);
		
		String class_name = javaClass.toString();
		
		try {
			Class<?> serviceClass = Class.forName(class_name);
			@SuppressWarnings("unchecked")
			Constructor<Function> c = (Constructor<Function>)serviceClass.getConstructor();
			Function f = c.newInstance();
			
			if ( config != null && (f instanceof ConfigurableFunction)) {
				((ConfigurableFunction)f).configure(config.asCollection().getMap(), this.fReg);
			}

//			Term fName = Term.tuple(module, name);
			Term fName = module.asSym().concat(name.asSym());
			
			this.fReg.addFunction(fName, f);
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
