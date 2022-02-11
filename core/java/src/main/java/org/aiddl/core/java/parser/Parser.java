package org.aiddl.core.java.parser;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.stream.Collectors;

import org.aiddl.core.java.container.Container;
import org.aiddl.core.java.function.DefaultFunctions;
import org.aiddl.core.java.function.FunctionRegistry;
import org.aiddl.core.java.tools.TermComparator;
import org.aiddl.core.java.parser.generated.AIDDLParser;
import org.aiddl.core.java.parser.generated.ParseException;
import org.aiddl.core.java.representation.StringTerm;
import org.aiddl.core.java.representation.Term;

/**
 * Parses AIDDL files into a container.
 * 
 * @author Uwe Koeckemann
 *
 */
public class Parser {

	private Parser() {}
	
	static FileInputStream fInStream; 
	
	private static String currentFolder;
	
	private static Term currentModule = Term.sym("default");
	
	private static boolean open = false;
	
	private static boolean verbose = false;
	
	private static List<String> parsedFiles = new LinkedList<>();
	
	private static Map<Term, String> moduleFileMap = new HashMap<>();
	private static Map<Term, String> modulePathMap = new HashMap<>();
	
	private static Map<String, Term> pathModuleMap = new HashMap<>();
	
	/** 
	 * Set the name of the module the parser is currently working on
	 * @param s name of the module
	 */
	public static void setCurrentModule( Term s ) {
		currentModule = s;
	}
	
	/** 
	 * Get the name of the module the parser is currently working on
	 * @return name of the current module
	 */
	public static Term getCurrentModule() {
		return currentModule;
	}
	
	/** 
	 * Set current working folder of parser for local references.
	 * @param s string representing the folder
	 */
	public static void setCurrentFolder( String s ) {
		currentFolder = s;
	}
	
	/**
	 * Get current working folder of parser for local references.
	 * @return folder parser is currently working in
	 */
	public static String getCurrentFolder() {
		return currentFolder;
	}
	
	/**
	 * Set verbose flag
	 */
	public static void setVerbose( boolean flag ) {
		verbose = true;
	}
	
	public static void reset() {
		parsedFiles.clear();
		moduleFileMap.clear();
		modulePathMap.clear();
		pathModuleMap.clear();
	}
	
	private static void loadAiddlPath( ) {
		String aiddlPathStr = System.getenv("AIDDL_PATH");
		String folderList[];
		
		String OS = System.getProperty("os.name");
		
		if ( OS.toLowerCase().contains("win") ) 
			folderList = aiddlPathStr.split(";");
		else
			folderList = aiddlPathStr.split(":");
		
		List<String> fileList;
		try {
			for ( String folderName : folderList ) {
				 fileList = Files.walk(Paths.get(folderName))
						.filter(Files::isRegularFile).map(x -> x.toString())
						.collect(Collectors.toList());
		
				for ( String fName : fileList ) {
					if ( fName.contains(".aiddl") && !fName.contains("~") && !fName.contains("#") ) {
						File f = new File(fName);
						Scanner scanner = new Scanner(f);
						scanner.useDelimiter("\\Z");			
						String content = scanner.next();
						scanner.close();
						String moduleName = null; // = content.split("\\(")[1].split("\\)")[1];
						String[] tmp = content.split("\n");
						for ( int i = 0 ; i < tmp.length ; i++ ) {
							if ( !tmp[i].trim().equals("") ) {
								String s = tmp[i];
								s = s.replace("\t", " ");
								while ( s.contains("  ") ) {
									s = s.replace("  ", " ");
								}
								moduleName = s.split(" ")[2];
								moduleName = moduleName.replace(")","").trim();
								break;
							}
						}
						if ( verbose )
							System.out.println(f.getAbsolutePath() + " -> " + moduleName);
						moduleFileMap.put(Term.sym(moduleName), f.getAbsolutePath());
					}
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.exit(0);
		} catch (IOException e1) {
			e1.printStackTrace();
			System.exit(0);
		}
	}
	
	public static Term getModuleFromFile( String fname ) {
		return Parser.pathModuleMap.get(fname);
	}
	
	public static boolean isKnownModule( Term modName ) {
		if ( moduleFileMap.isEmpty() ) {
			loadAiddlPath();
		}
		return moduleFileMap.containsKey(modName);
	}
	
	public static String getModuleFilename( Term modName ) {
		if ( moduleFileMap.isEmpty() ) {
			loadAiddlPath();
		}
		String fname = moduleFileMap.get(modName);
		if ( fname == null ) {
			List<Term> keys = new ArrayList<>();
			keys.addAll(moduleFileMap.keySet());
			Collections.sort(keys, new TermComparator());
			System.err.println("Known modules:");
			for ( Term k : keys ) {
				System.err.println(k + " -> " + moduleFileMap.get(k));
			}
			throw new IllegalArgumentException("Unknown module: " + modName + " in " + Parser.currentModule + " in " + Parser.currentFolder );
		}
		return moduleFileMap.get(modName);
	}
	
	public static StringTerm getModuleFolder( Term modName ) {
		String modFile = modulePathMap.get(modName);
		if ( modFile == null )
			throw new IllegalArgumentException("Requesting folder of unknown module: " + modName);
		return Term.string(modFile);
	}
		
	
	/**
	 * Set current expression as open or closed. 
	 * @param toggle use <code>true</code> to open and <code>false</code> to close
	 */
	public static void setOpenExpression( boolean toggle ) {
		open = toggle;
	}
	
	/** Parse a single term from a string.
	 * @param aiddlStr a string of AIDDL
	 * @return parsed term
	 */
	public static Term ParseTerm( String aiddlStr ) {
		Term currentModule = Parser.currentModule;
		Parser.currentFolder = null;
		InputStream inStream = new ByteArrayInputStream(aiddlStr.getBytes());
		
		FunctionRegistry freg = DefaultFunctions.createDefaultRegistry(new Container());
		
		AIDDLParser aiddlParser = new AIDDLParser(inStream);
		try {
			Term r = aiddlParser.Term( new Container( false ), freg );
			Parser.currentModule = currentModule;
			return r;
		} catch (ParseException e) {
			System.err.println("In string:\n" + aiddlStr);
			e.printStackTrace();
			System.exit(1);
		}
		Parser.currentModule = currentModule;
		return null;
	}
	
	/** Parse a single entry from a string into a container.
	 * @param aiddlStr a string of AIDDL representing an entry (i.e., a tuple of length 3 for type, name and value)
	 * @param db container to put the entry into
	 * @return <code>true</code> if the entry was parsed successfully, <code>false</code> otherwise 
	 */
	public static boolean ParseEntryIntoDatabase( String aiddlStr, Container db, FunctionRegistry freg ) {
		aiddlStr = aiddlStr.substring(1, aiddlStr.length()-1);
		
		InputStream inStream = new ByteArrayInputStream(aiddlStr.getBytes());
		
		AIDDLParser aiddlParser = new AIDDLParser(inStream); 
		try {
			aiddlParser.Entry(null, db, freg);
			return true;
		} catch (ParseException e) {
			System.err.println("In string:\n" + aiddlStr);
			e.printStackTrace();
			System.exit(1);
		}
		return false;
	}
	
	
	/** Parse a string into a database
	 * @param aiddlStr string of AIDDL representing a module
	 * @param db container to add the module to
	 */
	public static void parseString( String aiddlStr, Container db, FunctionRegistry freg ) {
		InputStream inStream = new ByteArrayInputStream(aiddlStr.getBytes());
		
		AIDDLParser aiddlParser = new AIDDLParser(inStream); 
		//aiddlParser.setVerbose(Parser.verbose);
		try {
			aiddlParser.DomainDefinition(db, freg);
		} catch (ParseException e) {
			System.err.println("In string:\n" + aiddlStr);
			e.printStackTrace();
			System.exit(1);
		}
	}
 	
	/**
	 * Parse a list of files. 
	 * @param db container to add modules to
	 * @param filenames to be parsed
	 */
	public static void parseFiles( String[] filenames, Container db, FunctionRegistry freg ) {
		for ( String filename : filenames ) {
			parseFileInternal(filename, db, freg);
		}			
		db.toggleNamespaces(true);
		freg.loadContainerDefintions(db);
		freg.loadContainerInterfaces(db);
		freg.loadTypeFunctions(db);
	}
	
	public static Term parseFile( String filename, Container db, FunctionRegistry freg  )  {
		Term name = parseFileInternal(filename, db, freg);
		db.toggleNamespaces(true);
		freg.loadContainerDefintions(db);
		freg.loadContainerInterfaces(db);
		freg.loadTypeFunctions(db);
		return name;
	}
		
	/** Parse a single file.
	 * @param filename name of file to be parsed
	 * @param db container to add the module to
	 * @return name of module contained in parsed file
	 */
	public static Term parseFileInternal( String filename, Container db, FunctionRegistry freg  )  {
		if ( moduleFileMap.isEmpty() ) {
			loadAiddlPath();
		}
		
		String OS = System.getProperty("os.name");
		
		String sep;
		if ( OS.toLowerCase().contains("win") ) 
			sep = ";";
		else
			sep = ":";
		

		try {
			String aiddlPathStr = System.getenv("AIDDL_PATH");

			File file = new File(filename);
			Scanner scanner = null;
			
			if ( !file.exists() ) {
				List<String> paths = new ArrayList<>();
				paths.add(currentFolder);
				if ( aiddlPathStr != null ) {
					for ( String s : System.getenv("AIDDL_PATH").split(sep) )
						paths.add(s);
				} 
				for ( String path : paths ) {
					file = new File(path + "/" +filename);
					if ( file.exists() ) {
						filename = path + "/" +filename;
						break;
					}
				}
			}
			if ( !file.exists() ) {
				for ( String fname : parsedFiles ) {
					System.err.println(fname);
				}
				throw new FileNotFoundException("Could not find " + filename + " locally or in AIDDL_PATH (" + System.getenv("AIDDL_PATH") + ")");
			}
			
//			if ( parsedFiles.contains(filename) ) {
//				if ( verbose ) {
//					System.out.println("Aleady parsed: " + filename);
//				}	
//				return pathModuleMap.get(filename);
//			} else {
				parsedFiles.add(filename);
				if ( verbose ) {
					System.out.println("Parsing: " + filename);
				}	
			
				scanner = new Scanner(file);
				
				scanner.useDelimiter("\\Z");			
				String content = scanner.next();
				scanner.close();
				
				String prevFolder = currentFolder;
				currentFolder = file.getParent();
						
				InputStream inStream = new ByteArrayInputStream(content.getBytes());
				
				AIDDLParser aiddlParser = new AIDDLParser(inStream);
				//aiddlParser.setVerbose(Parser.verbose);
				
				Term moduleName = aiddlParser.DomainDefinition(db, freg);
				
				Parser.modulePathMap.put(moduleName, file.getAbsoluteFile().getParent() + "/");
				Parser.moduleFileMap.put(moduleName, file.getAbsolutePath());
				Parser.pathModuleMap.put(filename, moduleName);
				
				if ( open ) {
					throw new ParseException("Unclosed expression in: " + file);
				}
				currentFolder = prevFolder;

				return moduleName;
		} catch (ParseException e) {
			System.err.println("In file " + filename);
			e.printStackTrace();
			System.exit(1);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.exit(1);
		}  
		return null;
	}
}
