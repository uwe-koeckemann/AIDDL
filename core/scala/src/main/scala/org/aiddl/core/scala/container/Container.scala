package org.aiddl.core.scala.container

import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.function.{Evaluator, Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger

import java.io.PrintWriter
import java.util.logging.Level
import scala.collection.mutable
import scala.collection.mutable.{HashMap, Map}

/**
 * Main data structure of the AIDDL core. Contains modules (usually parsed from .aiddl files) and a registry of all
 * known functions.
 */
class Container extends Verbose {

    private val funReg: mutable.Map[Sym, Function] = new mutable.HashMap
    private val entReg: mutable.Map[Sym, mutable.Map[Term, Entry]] = new mutable.HashMap
    private val entList: mutable.Map[Sym, List[Entry]] = new mutable.HashMap
    private var modList: List[Sym] = List.empty
    private val obsReg: mutable.Map[Sym, mutable.Map[Term, List[Function]]] = new mutable.HashMap
    private val aliasReg: mutable.Map[(Sym, Sym), Sym] = new mutable.HashMap
    private val interfaceReg: mutable.Map[Sym, Term] = new mutable.HashMap
    private val specialTypes: Set[Term] = Set(Sym("#type"), Sym("#def"), Sym("#req"), Sym("#nms"), Sym("#namespace"), Sym("#interface"), Sym("#mod"))

    private var verboseFunctions: List[Verbose] = Nil

    Function.loadDefaultFunctions(this)

    override def logGetVerboseSubComponents: List[Verbose] = this.verboseFunctions

    /**
     * Check if the container has a function
     * @param uri unique name of the function
     * @return <code>true</code> if the function exists, <code>false</code> otherwise
     */
    def hasFunction(uri: Sym): Boolean = funReg.contains(uri)

    /**
     * Add a new function
     * @param uri the name of the function
     * @param f the function
     */
    def addFunction(uri: Sym, f: Function): Unit = {
        if (f.isInstanceOf[Verbose]) {
            this.verboseFunctions = f.asInstanceOf[Verbose] :: this.verboseFunctions
        }
        this.funReg.put(uri, f)
    }

    /**
     * Get a function if it exists.
     * @param uri the name of the function
     * @return option containing the function if it exists, <code>None</code> otherwise
     */
    def getFunction(uri: Sym): Option[Function] = funReg.get(uri)

    /**
     * Get a reference to a function if it exists.
     * @param uri the name of the function
     * @return option containing the function reference if it exists, <code>None</code> otherwise
     */
    def getFunctionRef(uri: Sym):Option[FunRef] = funReg.get(uri).flatMap( f => Some(FunRef(uri, f)) )

    /**
     * Get a function if it exists or a default function otherwise.
     * @param uri the name of the function
     * @param default default function
     * @return the function if it exists, <code>default</code> otherwise
     */
    def getFunctionOrDefault(uri: Sym, default: Function):Function = funReg.getOrElse(uri, default)

    /**
     * Get a reference to a function if it exists or a reference to a default function otherwise.
     * @param uri the name of the function
     * @param default default function reference
     * @return the function reference if it exists, <code>default</code> otherwise
     */
    def getFunctionRefOrDefault(uri: Sym, default: FunRef): FunRef = funReg.get(uri).flatMap(f => Some(FunRef(uri, f))) match {
        case Some(value) => value
        case None => default
    }

    /**
     * Get a function if it exists and throws an exception otherwise
     * @param uri the name of the function
     * @return the function
     */
    def getFunctionOrPanic(uri: Sym): Function = {
        funReg.get(uri) match
            case Some(f) => f
            case None =>
                val stringBuilder = new StringBuilder()
                funReg.foreach(uri => stringBuilder.append(s"\n\t$uri"))
                throw new IllegalArgumentException(s"Function not registered: $uri. The following URIs are known:${stringBuilder.toString()}")
    }

    /**
     * Get a reference to a function if it exists and throws an exception otherwise
     * @param uri the name of the function
     * @return the function
     */
    def getFunctionRefOrPanic(uri: Sym): FunRef =
        FunRef(uri, getFunctionOrPanic(uri))

    /**
     * Add an interface definition.
     * @param uri the name of the interface
     * @param it term that defines the interface
     */
    def addInterfaceDef( uri: Sym, it: Term ): Unit = {
        interfaceReg.update(uri, it)
    }

    /**
     * Get interface defined under uri
     * @param uri name of the interface
     * @return interface definition term
     */
    def interface(uri: Sym): Term =
        this.interfaceReg(uri)

    /**
     * Get the evaluator function
     * @return evaluator registered in this container
     */
    def eval: Evaluator =
        funReg(EVAL).asInstanceOf[Evaluator]

    /**
     * Add a new module if a module under the given name does not exist. If the module exists, there is no change.
     *
     * @param module the name of the module
     */
    def addModule(module: Sym, alias: Sym = Sym("self")): Unit = {
        if (!entReg.contains(module)) {
            entReg.put(module, new mutable.HashMap)
            entList.put(module, Nil)
            modList = module :: modList
            this.addModuleAlias(module, alias, module)
            this.setEntry(module, Entry(Sym("#mod"), alias, module))
        }
    }

    /**
     * Get a list of all registered module names
     * @return list of module names
     */
    def getModuleNames: List[Sym] = modList

    /**
     * Save a module to a file
     * @param module name of the module
     * @param name filename to write module to
     */
    def saveModule(module: Sym, name: String): Unit = {
        //val modLine = s"(#mod ${this.findSelfAlias(module)} $module)"
        val sb = new mutable.StringBuilder
        //sb.append(modLine)
        //sb.append("\n\n")

        this.entList(module)
          .reverse
          .foreach( e => {
            sb.append(Logger.prettyPrint(e.asTuple, 0))
            sb.append("\n\n")
        })

        new PrintWriter(name) { write(sb.toString()); close() }
    }

    private def callObservers(module: Sym, entry: Entry) = {
        val obs = obsReg.getOrElse(module, mutable.Map.empty).getOrElse(entry.name, List())
        obs.map(_ apply entry.value)
    }

    /**
     * Set an entry in a module. If the module does not exist, it will be created.
     * @param module name of the module
     * @param entry the entry to set
     * @return option value containing entry that was overwritten or <code>None</code> if an entry under this name did
     *         not exist before
     */
    def setEntry(module: Sym, entry: Entry): Option[Entry] = {
        addModule(module)
        callObservers(module, entry)
        (entReg.get(module): @unchecked) match { 
            case Some(m) =>
                m.get(entry.name) match {
                    case Some(e) => entList.put(module, entry :: entList(module).filter(x => x != e))
                    case _ => entList.put(module, entry :: entList(module))
                }
                //if ( m.contains(entry.n) ) {}
                m.put(entry.name, entry)
        }
    }

    /**
     * Delete an entry from a module
     * @param module the module
     * @param entry entry to delete
     * @return option value containing deleted entry if it existed, <code>None</code> otherwise
     */
    def deleteEntry(module: Sym, entry: Entry): Option[Entry] = {
        callObservers(module, entry)
        entReg.getOrElse(module, mutable.Map.empty).remove(entry.name)
    }

    /**
     * Get an entry by its name from a module.
     * @param module name of the module
     * @param name name of the entry
     * @return option value containing the entry, <code>None</code> otherwise
     */
    def getEntry(module: Sym, name: Term):Option[Entry] =
        entReg.get(module) match {
            case Some(m) => m.get(name)  match {
                case Some(e) => Some(e)
                case None => m.find( (k, e) => k unifiable name ).flatMap( t => Some(t(1) \ (t(0) unify name).get ) )
            }
            case None => None }

    /**
     * Get all entries with a matching name, module, and type
     * @param modulePattern pattern for the module name
     * @param typePattern pattern for the type
     * @param namePattern pattern for the name
     * @return list of all entries in the container that match all patterns
     */
    def getMatchingEntries(modulePattern: Term, typePattern:Term, namePattern: Term): List[Entry] = {
        entList.filter( { case (k, _) => modulePattern unifiable k } )
               .values
               .flatMap( m => m.filter( { case Entry(t, n, _) => (namePattern unifiable n) && (typePattern unifiable t) } ) )
               .toList.reverse
    }

    /**
     * Get the value of an entry after using the evaluator to process it and resolve all references
     * @param module name of the module
     * @param name name of the entry
     * @return option value containing the processed value, <code>None</code> otherwise
     */
    def getProcessedValue(module: Sym, name: Term): Option[Term] =
        this.getEntry(module, name).flatMap( e => Some(this.eval.evalAllRefs(this.resolve(e.value))) )
    /**
     * Get the value of an entry after using the evaluator to process it and resolve all references.
     * Throws an exception if the module does not contain an entry with the given name
     * @param module name of the module
     * @param name name of the entry
     * @return the processed value
     */
    def getProcessedValueOrPanic(module: Sym, name: Term): Term =
        this.getEntry(module, name) match {
            case Some(e) => this.eval.evalAllRefs(e.value)
            case None => throw new NoSuchElementException(s"Module $module does not have an entry named $name")
        }

    /**
     * Get all entries associated to a given module
     * @param m name of the module
     * @return list of entries in <code>m</code>
     */
    def getModuleEntries(m: Sym): List[Entry] = this.entList.getOrElse(m, Nil)

    /**
     * Resolve entry references in a term.
     * @param t the term
     * @return a term with all entry references replaced by the values of their entries
     */
    def resolve( t: Term ): Term = t match {
        case Tuple(args @ _*) => Tuple(args.map(x => this.resolve(x)): _*)
        case ListTerm(list) => ListTerm(list.map(x => this.resolve(x)))
        case SetTerm(set) => SetTerm(set.map(x => this.resolve(x)))
        case KeyVal(key, value) => KeyVal(this.resolve(key), this.resolve(value))
        case r @ EntRef(_, _, _) => resolve(resolveReference(r))
        case _ => t
    }

    /**
     * Resolve a single entry reference. Follows references chains until the result is no longer an entry reference term.
     * @param r entry reference
     * @return non entry reference term
     */
    def resolveReference( r: EntRef ): Term = {
        resolveReferenceOnce(r) match {
            case Some(next @ EntRef(_, _, _)) if next == r => next
            case None => throw new IllegalArgumentException(s"Cannot resolve reference: $r target=${r.mod} (class: ${r.getClass.getSimpleName})" )
            case Some(next @ EntRef(_, _, _)) => resolveReference(next)
            case Some(v) => v
        }
    }

    private def resolveReferenceOnce(r: EntRef): Option[Term] = {
        if r.name.isInstanceOf[EntRef] then Some(r.name)
        else
            this.getEntry(this.findModuleAlias(r.mod, r.alias), r.name) match
                case Some(Entry(Sym("#def"), _, _)) => Some(r)
                case Some(Entry(_, _, v)) => Some(v)
                case _ => None
    }

    /**
     * Add an alias for another module to a module
     * @param module module that uses the alias
     * @param alias the alias
     * @param target the target module
     */
    def addModuleAlias(module: Sym, alias: Sym, target: Sym): Unit = {
        this.aliasReg((module, alias)) = target
    }

    /**
     * Get the actual name for a module alias
     * @param source module that uses the alias
     * @param alias the alias
     * @return the global name referred to by the alias
     */
    def findModuleAlias(source: Sym, alias: Sym):Sym = {
        this.aliasReg.get((source, alias)) match {
            case Some(uri) => uri
            case None => throw new IllegalArgumentException(s"Alias $alias not defined in module $source.\nAliases are added with requirement entries such as: (#req $alias MODULE), where MODULE is the symbolic name of a module or a relative filename.")

        }
    }

    /**
     * Find the self-alias of a module. The self alias is established in the #mod entry of a module. If the #mod entry is
     * <code>(#mod self name)</code>, the module <code>name</code> refers to itself with the symbol <code>self</code>.
     * @param source the module
     * @return the module's self alias
     */
    def findSelfAlias(source: Sym):Sym =
        this.aliasReg.find((k, target) => k(0) == target).get(0)(1)

    /**
     * Check if entries have the correct types in all modules.
     * @param verbose flag to generate some debug output
     * @return <code>true</code> if all entry values satisfy their given types, <code>false</code> otherwise
     */
    def typeCheckAllModules(): Boolean = {
        modList.forall(m => {
            this.logger.info(s"Module: $m")
            typeCheckModule(m)
        })
    }

    /**
     * Type check all entries in a single module
     * @param uri name of the module
     * @param verbose turn on debug messages
     * @return <code>true</code> if all entry values satisfy their given types, <code>false</code> otherwise
     */
    def typeCheckModule(uri: Sym, verbose: Boolean = false): Boolean = {
        entList.get(uri) match {
            case Some(es) =>
                es.forall( e => {
                    val isConsistent = checkSingleType(uri, e.typeRef, e.value)
                    this.logger.info(s"$uri | ${e.typeRef} | ${e.name} | $isConsistent ")
                    isConsistent
                })
            case None =>
                val sb = new StringBuilder()
                sb.append(s"There are ${modList.length} registered modules:")
                modList.foreach(m => sb.append(s"\n\t$m"))
                throw new IllegalArgumentException(s"Module $uri does not exist.\n${sb.toString()}")
        }
    }

    private def checkSingleType(m: Sym, t: Term, v: Term): Boolean = {
        try {
            eval(t) match {
                case _: FunRef => t(this.eval(resolve(v))).asBool.v
                case s: Sym if this.specialTypes contains s => true
                case furi: Sym => this.getFunctionOrPanic(furi)(this.eval(resolve(v))).asBool.v
                case _ =>
                    val hint = if (!t.isInstanceOf[EntRef]) "" else s"\nHint: If this entry reference corresponds to a type, try using ^$t instead (adding the ^) to make it a function reference."
                    throw new IllegalArgumentException(s"Bad type specifier $t in module $m. Use symbolic type URI or function reference instead.$hint")
            }
        } catch {
            case ex: Throwable => {
                ex.printStackTrace()
                throw new IllegalArgumentException(s"Error when checking type ${t} in module $m with value $v.")
            }
        }
    }

    override def toString: String =
        this.modList
          .map(m => this.entList(m).reverse.mkString("", "\n", ""))
          .reverse
          .mkString("", "\n", "")
}

