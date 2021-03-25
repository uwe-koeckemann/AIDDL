package org.aiddl.core.container;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

import org.aiddl.core.interfaces.Module;
import org.aiddl.core.interfaces.Observer;
import org.aiddl.core.representation.ReferenceTerm;
import org.aiddl.core.representation.Substitution;
import org.aiddl.core.representation.SymbolicTerm;
import org.aiddl.core.representation.Term;
import org.aiddl.core.representation.TupleTerm;
import org.aiddl.core.tools.Logger;

/** 
 * Implementation of {@link Container}. Main access point to read and write AIDDL entries.
 * Each entry belongs to a module (identified by a name term). Operations that do not 
 * state a module usually use the working module which, by default, is the module of the
 * first file loaded into the container. 
 * 
 * @author Uwe Köckemann
 */
public class Container {
	
	private Module workingModule = null;
	
	private Map<Term,Module> modules;
	private List<Module> moduleList;

	Map<Term,Map<Term,Term>> aliasLookup;
	Map<Term,Term> selfAliasLookup;
	
	boolean threadSafe = false;
	
	/**
	 * Create an empty container.
	 */
	public Container() {
		this(false);
	}
	
	/**
	 * Create an empty, optionally thread safe, container.
	 * @param threadSafe set to <code>true</code> to use thread safe data structures internally
	 */
	public Container( boolean threadSafe ) {
		if ( threadSafe ) {
			this.threadSafe = true;
			modules = new ConcurrentHashMap<>();
			moduleList = new CopyOnWriteArrayList<>();

			aliasLookup = new ConcurrentHashMap<>();// HashMap<>();
			selfAliasLookup = new ConcurrentHashMap<>();
		} else {
			modules = new HashMap<>();
			moduleList = new ArrayList<>();

			aliasLookup = new HashMap<>();// HashMap<>();
			selfAliasLookup = new HashMap<>();
		}
	}
	
	public void switchModuleToThreadSafe( SymbolicTerm modName ) {
		Module m = this.modules.get(modName);
		if ( m instanceof LocalModule ) {
			((LocalModule)m).switchToThreadSafe();
		}
	}
		

	public Entry getEntry( Term name ) {
		for ( Entry e : workingModule.getEntries() ) {
			Substitution s = e.getName().match(name);
			if ( s != null ) {
				return e.substitute(s);
			}
		}
		return null;
	}
	

	public Entry getEntry( Term modName, Term name ) {
		Module m = modules.get(modName);
		if ( m == null ) {
			System.err.println(modName);
			System.err.println(name);
			System.err.println(this.aliasLookup);
			System.err.println(this.modules);
			throw new IllegalArgumentException("Requesting entry from unknown module: " + modName);
		}
		for ( Entry e : m.getEntries() ) {
			Substitution s = e.getName().match(name);
			if ( s != null ) {
				return e.substitute(s);
			}
		}
		return null;
	}
	

	public Collection<Entry> getMatchingEntries( Term modulePattern, Term typePattern, Term namePattern ) {
		ArrayList<Entry> r = new ArrayList<Entry>();
		for ( Module m : this.moduleList ) {
			Substitution s_base = null;
			if ( modulePattern != null ) {
				s_base = modulePattern.match(m.getName());
			} else {
				s_base = new Substitution();
			}
			if ( s_base != null ) {
				for ( Entry e : m.getEntries() ) {
					Substitution s = s_base.copy();
					
					if ( typePattern != null ) {
						Substitution s_tmp = typePattern.match(e.getType());
						if ( s_tmp == null || !s.add(s_tmp)) {
							continue;
						}
					}
					if ( namePattern != null ) {
						Substitution s_tmp = namePattern.match(e.getName());
						if ( s_tmp == null || !s.add(s_tmp)) {
							continue;
						} else {
							r.add(e);
						}
					} else if ( s != null ) {
						r.add(e);
					}
				}
			}
		}
		return r;
	}
	

	public List<Term> getModuleNames() {
		List<Term> moduleList = new ArrayList<>();
		for ( Module m : this.moduleList ) {
			moduleList.add(m.getName());
		}
		return moduleList;
	}

	

	public void setEntry( Entry entry ) {
		if ( !entry.getName().resolve(this).equals(entry.getName()) ) {
			throw new IllegalArgumentException("Entry name " + entry.getName() + " has references. Make sure to resolve references before creating the entry." );
		}
		
		Entry prev = this.getEntry(entry.name);
		if ( prev != null ) {
			workingModule.removeEntry(prev);
		}
		workingModule.putEntry(entry);
	}
	

	public void setEntry( Term module, Entry entry ) {
		Module mSet = this.modules.get(module);
		if ( mSet == null ) {
			throw new IllegalArgumentException("Module " + module + " does not exist.");
		}
		if ( !entry.getName().resolve(this).equals(entry.getName()) ) {
			throw new IllegalArgumentException("Entry name " + entry.getName() + " has references. Make sure to resolve references before creating the entry." );
		}
		
		Entry prev = this.getEntry(entry.name);
		if ( prev != null ) {
			mSet.removeEntry(prev);
		}
		mSet.putEntry(entry);
	}
	

	public void copyEntry( SymbolicTerm module_a, Term a, SymbolicTerm module_b, Term b ) {
		Entry aEntry = this.getEntry(module_a, a);
		this.setEntry(module_b, new Entry(aEntry.getType(),aEntry.getName(), aEntry.getValue()));
	}


	public void deleteEntry(Entry entry) {
		workingModule.removeEntry(entry);
	}

	public void deleteEntry(Term module, Entry entry) {
		Module mSet = this.modules.get(module);

		if ( mSet == null ) {
			throw new IllegalArgumentException("Module " + module + " does not exist.");
		}
		mSet.removeEntry(entry);
	}


//	public void compileAndAdd(String s) {
//		Parser.parseString(s, this);
//	}
	

	public void export( Term moduleName, String filename ) {
		Module m = this.modules.get(moduleName);
		if ( m == null ) {
			throw new IllegalArgumentException("Requesting non-existing module: " +  moduleName);
		}
		FileWriter fstream;
		try {
			fstream = new FileWriter(filename);

			BufferedWriter out = new BufferedWriter(fstream);
			//out.write("(#mod self " + m.getName()+ ")" );
			out.write("\n");
			for ( Entry e : m.getEntries() ) {
				out.write("(\n");
				out.write(Logger.prettyPrint(e.getType(), 1));
				out.write("\n");
				out.write(Logger.prettyPrint(e.getName(), 1));
				out.write("\n");
				out.write(Logger.prettyPrint(e.getValue(), 1));
				out.write("\n)\n\n");
			}
			out.close();
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	

	public Term resolveReference( ReferenceTerm t ) {
		Term next = t; 
		Term current = t;
		do {
			if ( next instanceof ReferenceTerm ) {
				current = next;
				next = resolveReferenceOnce((ReferenceTerm)current);
				if ( current.equals(next) ) {
					return current;
				}
			} else {
				return next;
			}
		} while ( next != null );
		throw new IllegalArgumentException("Reference cannot be resolved: " + t + " refered module: " + t.asRef().getRefModule());
	}
	
	private Term resolveReferenceOnce( ReferenceTerm t ) {
		Term target = t.getRefTarget();
		if ( target instanceof ReferenceTerm ) {
			return target;
		}
		Term modName = t.getRefModule();
		Entry e =  this.getEntry(modName, target);
		if ( e == null ) {
			return null;
		}
		if ( e.getType().equals(Term.sym("#def")) ) {
			return t;
		}
		Term next = e.getValue();
		return next;
	}
	

	@Override
	public String toString() {
		StringBuilder sB = new StringBuilder();
		sB.append("Entries:\n");
		for ( Module m : this.moduleList ) {
			sB.append( "Module: " + m.getName() );
			if ( m == workingModule ) {
				sB.append(" <--- Working Module");
			}
			
			sB.append("\n");
			for ( Entry e : m.getEntries() ) {
				sB.append("\t");
				sB.append(e.toString());
				sB.append("\n");
			}
		}
		return sB.toString();
	}
	

	public Term getWorkingModule() {
		return this.workingModule.getName();
	}
	

	public void setWorkingModule( Term moduleName ) {
		Module m = this.modules.get(moduleName);
		if ( m == null ) {
			throw new IllegalAccessError("Trying use non-existing module " + moduleName + " as working module.");
		}
		this.workingModule = m;
	}


	public void addModule( Term name ) {
		LocalModule m = new LocalModule( name, this.threadSafe );
		if ( this.modules.putIfAbsent(name, m) == null ) {
			this.moduleList.add(m);
		}
		if ( workingModule == null ) {
			workingModule = m;
		}
	}
	

//	public void addProxyModule( SymbolicTerm name, SymbolicTerm nodeUri, SocketReceiver in, SocketSender out ) {
//		ProxyModule m = new ProxyModule(name, nodeUri, out, in);
//		if ( this.modules.putIfAbsent(name, m) == null ) {
//			this.moduleList.add(m);
//		}
//		if ( workingModule == null ) {
//			workingModule = m;
//		}
//		for ( Entry e : m.getEntries() ) {
//			if ( e.getType().equals(Term.sym("#mod") )) {
//				this.addModuleAlias(name, e.getName(), name);
//			} else if ( e.getType().equals(Term.sym("#req"))) {
//				Term refMod = e.getValue();
//				if ( refMod instanceof StringTerm ) {
//					refMod = Parser.getModuleFromFile(refMod.getStringValue());
//				}
//				this.addModuleAlias(name, e.getName(), refMod);
//			}
//		}
//	}
	
	public void removeModule( Term name ) {
		Module m = this.modules.get(name);
		this.modules.remove(name);
		this.moduleList.remove(m);
		if ( this.workingModule.equals(m) ) {
			this.workingModule = null;
		}
	}

	public void addModuleAlias(Term sourceModule, Term alias, Term targetModule) {
		if ( !aliasLookup.containsKey(sourceModule) ) {
			aliasLookup.put(sourceModule, new HashMap<>());
		}
		aliasLookup.get(sourceModule).put(alias, targetModule);
		if ( sourceModule.equals(targetModule) ) {
			this.selfAliasLookup.put(sourceModule, alias);
		}
	}


	public Term resolveModuleAlias(Term sourceModule, Term alias) {
		if ( this.aliasLookup.containsKey(sourceModule) ) {
			return this.aliasLookup.get(sourceModule).get(alias);
		} 
		return null;
	}
	

	public Term findSelfAlias(Term moduleName ) {
		return this.selfAliasLookup.get(moduleName);
	}


	public void registerObserver( Term module, Term entryName, Observer obs ) {
		Module m = this.modules.get(module);
		if ( m == null ) {
			throw new IllegalAccessError("Trying to add observer to non-existing module: " + module);
		}
		m.addObserver(entryName, obs);
	}
	
//
//	public void toggleFullFunctionNames( boolean flag ) {
//		for ( Module m : this.moduleList ) {
//			m.toggleFullFunctionNames();
//		}
//	}
	

	public void toggleNamespaces( boolean flag ) {
		Map<Term, Substitution> sub_map = new HashMap<>();
		for ( Module m : this.moduleList ) {
			Substitution ns_sub = m.getNamespaceSubstitution();
			if ( !flag ) 
				ns_sub = ns_sub.inverse();
			sub_map.put(m.getName(), ns_sub);
		}
		
		for ( Module m : this.moduleList ) {
			Collection<Entry> namespaces = this.getMatchingEntries(m.getName(), Term.sym("#namespace"), null);
			namespaces.addAll(this.getMatchingEntries(m.getName(), Term.sym("#nms"), null));
			Substitution s = new Substitution(true);
			for ( Entry ns : namespaces ) {
				Term ns_term = ns.getValue();
				if ( ns_term instanceof  TupleTerm ) {
					Term ns_name = ns_term.get(0);
					Term ns_table = this.resolveReference(ns_term.get(1).asRef());
					
//					System.out.println("Namespace: " + ns_name);
//					System.out.println("Table:\n" + Logger.prettyPrint(ns_table, 0));
					
					Substitution ns_sub = new Substitution();
					for ( Term entry : ns_table.asCollection() ) {
						ns_sub.add(entry.get(ns_name), entry.get(0));
					}
					
//					System.out.println(ns_sub);
					
					if ( !s.add(ns_sub) ) {
						System.err.println("Namespace term: " + ns_term);
						System.err.println("Substitution before: " + s);
						System.err.println("Substitution failed: " + ns_sub);
						throw new IllegalStateException("Name space conflict in module: " + m.getName() + "\nConsider using #req instead of #namespace and use references.");
					}
					
				} else {
					Term ns_mod_name = this.resolveModuleAlias(m.getName(), ns.getName());
					if ( !s.add(sub_map.get(ns_mod_name)) ) {
						System.err.println(sub_map);
						System.err.println(ns);
						System.err.println(s);
						System.err.println(ns_mod_name);
						System.err.println(sub_map.get(ns_mod_name));
						throw new IllegalStateException("Name space conflict in module: " + m.getName() + "\nConsider using #req instead of #namespace and use references.");
					}
				}
			}
			m.substitute(s);
		}
	}
}
