package org.aiddl.core.interfaces;

import java.util.Collection;

import org.aiddl.core.container.Entry;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.Term;

public interface Module {

	Term getName();

	void putEntry(Entry e);

	void removeEntry(Entry e);

	Collection<Entry> getEntries();

	void addObserver(Term entryName, Observer obs);

	Substitution getNamespaceSubstitution();

	void substitute(Substitution s);

//	void toggleFullFunctionNames();

}