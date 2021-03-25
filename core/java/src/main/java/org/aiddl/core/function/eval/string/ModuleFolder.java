package org.aiddl.core.function.eval.string;

import org.aiddl.core.interfaces.PureFunction;
import org.aiddl.core.parser.Parser;
import org.aiddl.core.representation.Term;

public class ModuleFolder implements PureFunction {

	@Override
	public Term apply(Term args) {
		return Parser.getModuleFolder(args);
	}
}
