package org.aiddl.core.container;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

import org.aiddl.core.interfaces.Function;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;

class Module {
	SymbolicTerm moduleUri;
	
	private static List<Function> NULL_LIST = new ArrayList<>();
	
	private Map<Term, Entry> data;
	private Map<Term, List<Function>> observerMap;
	
	boolean threadSafe = false;
	
	public Module( Term modID, boolean threadSafe ) {
		this.threadSafe = threadSafe;
		this.moduleUri = modID.asSym();
		if ( this.threadSafe ) {
			this.data = new ConcurrentHashMap<>();
			this.observerMap = new ConcurrentHashMap<>();
		} else {
			this.data = new LinkedHashMap<>();
			this.observerMap = new LinkedHashMap<>();
		}
		this.putEntry(new Entry(Term.sym("#mod"), Term.sym("self"), modID));
	}
	
	protected void switchToThreadSafe() {
		if ( !threadSafe ) {
			Map<Term, Entry> data = new ConcurrentHashMap<>();;
			Map<Term, List<Function>> observerMap = new ConcurrentHashMap<>();
			
			data.putAll(this.data);
			observerMap.putAll(this.observerMap);
			
			this.data = data;
			this.observerMap = observerMap;
		}
		
	}
	

	public Term getName() {
		return moduleUri;
	}
	

	public void putEntry( Entry e ) {
		this.data.put(e.getName(), e);
		for ( Function obs : this.observerMap.getOrDefault(e.getName(), NULL_LIST) ) {
			obs.apply(e.getValue());
		}
	}
	

	public  void removeEntry( Entry e ) {
		this.data.remove(e.getName());
	}
	

	public Collection<Entry> getEntries() {
		return data.values();
	}	
	

	public void addObserver( Term entryName, Function obs, boolean dataOnly ) {
		if ( threadSafe ) {
			this.observerMap.putIfAbsent(entryName, new CopyOnWriteArrayList<>());
		} else {
			this.observerMap.putIfAbsent(entryName, new ArrayList<>());
		}
		
		this.observerMap.get(entryName).add(obs);
	}
	
	private static Term MOD = Term.sym("#mod");

	public Substitution getNamespaceSubstitution() {
		Substitution name_sub = new Substitution(true);
		for ( Entry e : this.data.values() ) {
			if ( !e.getType().equals(MOD) ) {
				if ( !Substitution.isSelfReferentialSub(e.getName(), e.getValue())) {
					name_sub.add(e.getName(), e.getValue());
				}
			}
		}
		return name_sub;
	}
	

	public void substitute( Substitution s ) { //TODO: Delete old entries on name change
		List<Entry> subbed_entries = new ArrayList<>();
		for ( Entry e : this.data.values()  ) {
			subbed_entries.add( e.substitute(s) );
		}
		for ( Entry e : subbed_entries ) {
			this.putEntry(e);
		}
	}
	
	@Override
	public boolean equals( Object o ) {
		return this == o;
	}
	
	private static final SymbolicTerm DEF = Term.sym("#def"); 
//	private boolean fullFunctionNames = false;
//	private Substitution functionNameSub = null;
//	private Substitution functionNameSubInv = null;
	
	/* (non-Javadoc)
	 * @see org.aiddl.core.container.ModuleInterface#toggleFullFunctionNames()
	 */
//	@Override
//	public void toggleFullFunctionNames() {
//		if ( functionNameSub == null ) {
//			functionNameSub = new Substitution();
//			for ( Entry e : this.getEntries() ) {
//				if ( e.getType().equals(DEF) ) {
//					if ( e.getName() instanceof SymbolicTerm ) {
//						SymbolicTerm funUri = this.moduleUri.concat(e.getName().asSym());
//						functionNameSub.add(e.getName(), funUri);
//					}
//				}
//			}
//			functionNameSubInv = functionNameSub.inverse();
//		}
//		
//		System.out.println("MODULE: " + moduleUri);
//		System.out.println(functionNameSub);
//		if ( fullFunctionNames ) {
//			this.substitute(functionNameSubInv);
//		} else {
//			this.substitute(functionNameSub);
//		}
//	}
	
}
