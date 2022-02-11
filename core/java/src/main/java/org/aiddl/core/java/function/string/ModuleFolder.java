package org.aiddl.core.java.function.string;

import org.aiddl.core.java.interfaces.Function;
import org.aiddl.core.java.parser.Parser;
import org.aiddl.core.java.representation.Term;

public class ModuleFolder implements Function {

	@Override
	public Term apply(Term args) {
		return Parser.getModuleFolder(args);
	}
}
