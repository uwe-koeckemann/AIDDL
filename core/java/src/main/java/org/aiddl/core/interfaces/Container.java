//package org.aiddl.core.interfaces;
//
//import java.util.Collection;
//import java.util.List;
//
//import org.aiddl.core.container.Entry;
//import org.aiddl.core.network.SocketReceiver;
//import org.aiddl.core.network.SocketSender;
//import org.aiddl.core.representation.ReferenceTerm;
//import org.aiddl.core.representation.SymbolicTerm;
//import org.aiddl.core.representation.Term;
//
///**
// * Interface for data structure containing AIDDL entries in different modules.
// * @author Uwe Köckemann
// *
// */
//public interface Container {
//	
//	/**
//	 * Add a new, empty module. If a module with the given name exists, there is no change.
//	 * @param name name of added module
//	 */
//	void addModule( Term name );
//	
//	
//	public void addProxyModule( SymbolicTerm name, SymbolicTerm nodeUri, SocketReceiver in, SocketSender out );
//		
//	
//	/**
//	 * Get {@link Entry} by name from the working module. Does not perform matching.
//	 * @param name Name of the requested entry.
//	 * @return entry with requested name if it exists, <code>null</code> otherwise
//	 */
//	Entry getEntry ( Term name );
//	/**
//	 * Get {@link Entry} by name from module. Does not perform matching.
//	 * @param moduleName name of module with entry
//	 * @param name name of the entry
//	 * @return entry with requested name if it exists, <code>null</code> otherwise
//	 */
//	Entry getEntry ( Term moduleName, Term name );
//	
//	/**
//	 * Get entries matching a module, type and name pattern.
//	 * @param modulePattern 
//	 * @param typePattern 
//	 * @param namePattern
//	 * @return all matching entries and (if <code>includeReferredTerms</code> is <code>true</code>) all referred entries
//	 */
//	Collection<Entry> getMatchingEntries ( Term modulePattern, Term typePattern, Term namePattern );
//	
//	/**
//	 * Get names of all modules in this container
//	 * @return
//	 */
//	public List<Term> getModuleNames();
//	
//	/** 
//	 * Add or update a single entry in working module
//	 * @param entry entry to update
//	 */
//	void setEntry ( Entry entry );
//
//	/**
//	 * Add or update a single entry in module
//	 * @param module module to change
//	 * @param entry entry to update
//	 */
//	void setEntry ( Term module, Entry entry );
//
//	/**
//	 * Copy entry with name <code>a</code> in module to entry with name <code>b</code>
//	 * @param module target module
//	 * @param a name of entry to copy
//	 * @param b target entry
//	 */
//	void copyEntry ( SymbolicTerm module_a, Term a, SymbolicTerm module_b, Term b );
//	
//	/**
//	 * Delete an entry from working module
//	 * @param entry entry to delete
//	 */
//	void deleteEntry ( Entry entry );
//	
//	/**
//	 * Delete an entry from working module
//	 * @param module target module
//	 * @param entry entry to delete
//	 */
//	void deleteEntry ( Term module,  Entry entry );
//		
//	/**
//	 * Add a string of AIDDL to the database (including type definitions)
//	 * @param s string of AIDDL code to compile
//	 */
//	void compileAndAdd ( String s );
//	
//	/**
//	 * Export module to file.
//	 * @param moduleName name of module
//	 * @param filename name of file
//	 */
//	void export( Term moduleName, String filename );
//			
//	/** Get name of current working module.
//	 * @return term representing the name of the current working module
//	 */
//	Term getWorkingModule();
//	
//	/**
//	 * Set working module 
//	 * @param moduleName name of new working module
//	 */
//	void setWorkingModule( Term moduleName );
//	
//	/**
//	 * Add alias for targetModule to source module 
//	 * @param sourceModule module that uses the alias
//	 * @param alias a short name used by <code>sourceModule</code> for <code>targetModule</code>
//	 * @param targetModule global name of module that the alias is used for
//	 */
//	void addModuleAlias( Term sourceModule, Term alias, Term targetModule );
//	
//	/**
//	 * Find global name for alias in sourceModule
//	 * @param sourceModule module that uses the alias
//	 * @param alias short name for global module
//	 * @return global module name
//	 */
//	Term resolveModuleAlias( Term sourceModule, Term alias );
//	
//	/**
//	 * Get alias a module uses for itself
//	 * @param module name of module
//	 * @return self alias of module
//	 */
//	Term findSelfAlias( Term module );
//	
//	/**
//	 * Resolve all references in a term.
//	 * @param t input term
//	 * @return term containing no references (except self references which are resolved once)
//	 */
//	Term resolveReference ( ReferenceTerm t );
//	
//	/**
//	 * Register an observer for an entry in a module
//	 * @param module name of module
//	 * @param entryName name of entry to observe
//	 * @param obs observer implementation that is called when the entry is updated
//	 */
//	void registerObserver( Term module, Term entryName, Observer obs );
//	
//	/**
//	 * If <code>true</code>, perform name replacements across all modules. 
//	 * If <code>false</code>, undo name replacements. 
//	 * @param flag
//	 */
////	void toggleFullFunctionNames( boolean flag );
//	
//	/**
//	 * If <code>true</code>, perform name space replacements accros all modules. 
//	 * If <code>false</code>, undo name space replacements. 
//	 * @param flag
//	 */
//	void toggleNamespaces( boolean flag );
//	
//	/**
//	 * Switch a module to use thread safe data structures
//	 * @param modName name of the module
//	 */
//	public void switchModuleToThreadSafe( SymbolicTerm modName );
//}
