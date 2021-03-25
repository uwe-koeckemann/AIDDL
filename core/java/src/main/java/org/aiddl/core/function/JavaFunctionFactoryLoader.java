package org.aiddl.core.function;

import java.lang.reflect.Constructor;
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
public class JavaFunctionFactoryLoader implements Function {
	
	private static SymbolicTerm FunctionName = Term.sym("name");
	private static SymbolicTerm ModuleName = Term.sym("module");
	private static SymbolicTerm JavaClass = Term.sym("class");

	FunctionRegistry fReg;
	
	public JavaFunctionFactoryLoader(FunctionRegistry fReg) {
		this.fReg = fReg;
	}
	
	@Override
	public Term apply(Term args) {
		Term name = args.get(FunctionName);
		Term module = args.get(ModuleName);		
		Term javaClass = args.get(JavaClass);
		
		String class_name = javaClass.toString();
		
		try {
			Class<?> serviceClass = Class.forName(class_name);
			@SuppressWarnings("unchecked")
			Constructor<Function> c = (Constructor<Function>)serviceClass.getConstructor();
			
			JavaFunctionFactory fact = new JavaFunctionFactory(c, fReg);
			
			Term fName = module.asSym().concat(name.asSym());		
			
			this.fReg.addFunction(fName, fact);
			return fName; //Term.fref(fName.asSym(), fReg);
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (SecurityException e) {
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
