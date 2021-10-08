package org.aiddl.core.function.string;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;

public class ModuleFolder implements Function {

	@Override
	public Term apply(Term args) {
		return Parser.getModuleFolder(args);
	}
}
