package org.aiddl.core.scala.container

import org.aiddl.core.scala.eval.Evaluator

import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.tools.Logger

import java.io.PrintWriter



class Container {

    val funReg: Map[Sym, Function] = new HashMap
    val entReg: Map[Sym, Map[Term, Entry]] = new HashMap
    val entList: Map[Sym, List[Entry]] = new HashMap
    var modList: List[Sym] = List.empty
    var obsReg: Map[Sym, Map[Term, List[Function]]] = new HashMap
    var aliasReg: Map[(Sym, Sym), Sym] = new HashMap
    var interfaceReg: Map[Sym, Term] = new HashMap

    val specialTypes: Set[Term] = Set(Sym("#type"), Sym("#def"), Sym("#req"), Sym("#nms"), Sym("#namespace"), Sym("#interface"), Sym("#mod"))

    Function.loadDefaultFunctions(this)

    def hasFunction(uri: Sym): Boolean = funReg.contains(uri)
    def addFunction(uri: Sym, f: Function) = this.funReg.put(uri, f)
    def getFunction(uri: Sym):Option[Function] = funReg.get(uri)
    def getFunctionRef(uri: Sym):Option[FunRef] = funReg.get(uri).flatMap( f => Some(FunRef(uri, f)) )
    def getFunctionOrDefault(uri: Sym, f: Function):Function = funReg.getOrElse(uri, f)
    def getFunctionRefOrDefault(uri: Sym, f: Function): FunRef = FunRef(uri, funReg.getOrElse(uri, f))
    def getFunctionOrPanic(uri: Sym): Function = {
        funReg.get(uri) match
            case Some(f) => f
            case None => {
                println("Registered functions:")
                funReg.foreach(println)
                throw new IllegalAccessError("Function not registered: " + uri)
            }
    }
    def getFunctionRefOrPanic(uri: Sym): FunRef = FunRef(uri, getFunctionOrPanic(uri))
    def addInterfaceDef( uri: Sym, it: Term ): Unit = { interfaceReg.update(uri, it) }
    def eval: Evaluator = funReg(EVAL).asInstanceOf[Evaluator]

    def addModule(module: Sym) = {
        if ( !entReg.contains(module) ) {
           entReg.put(module, new HashMap)
           entList.put(module, Nil)
           modList = module :: modList
        }
    }
    def getModuleNames: List[Sym] = modList

    def saveModule(module: Sym, fname: String) = {
        val modLine = s"(#mod ${this.findSelfAlias(module)} $module)"
        val sb = new StringBuilder
        sb.append(modLine)
        sb.append("\n\n")

        this.entList(module).foreach( e => {
            sb.append(Logger.prettyPrint(e.asTuple, 0))
            sb.append("\n\n")
        })

        new PrintWriter(fname) { write(sb.toString()); close }
    }

    private def callObservers(module: Sym, entry: Entry) = {
        val obs = obsReg.getOrElse(module, Map.empty).getOrElse(entry.n, List())
        obs.map(_ apply entry.v)
    }

    def setEntry(module: Sym, entry: Entry): Option[Entry] = {
        addModule(module)
        callObservers(module, entry)
        (entReg.get(module): @unchecked) match { 
            case Some(m) => {
                m.get(entry.n) match {
                    case Some(e) => entList.put(module, entry :: entList.get(module).get.filter(x => x != e)) 
                    case _ => { entList.put(module, entry :: entList.get(module).get) }
                }
                if ( m.contains(entry.n) ) {
                    
                }
                m.put(entry.n, entry) 
            } 
        }
    }
    
    def deleteEntry(module: Sym, entry: Entry): Option[Entry] = {
        callObservers(module, entry)
        entReg.getOrElse(module, Map.empty).remove(entry.n)
    }

    def getEntry(module: Sym, name: Term):Option[Entry] =
        entReg.get(module) match {
            case Some(m) => m.get(name)  match {
                case Some(e) => Some(e)
                case None => m.find( (k, e) => k unifiable name ).flatMap( t => Some(t(1) \ (t(0) unify name).get ) )
            }
            case None => None }

    def getMatchingEntries(modulePattern: Term, typePattern:Term, namePattern: Term): List[Entry] = {
        entList.filter( { case (k, _) => modulePattern unifiable k } )
               .values
               .flatMap( m => m.filter( { case Entry(t, n, _) => (namePattern unifiable n) && (typePattern unifiable t) } ) )
               .toList.reverse
    }

    def getProcessedValue(module: Sym, name: Term): Option[Term] =
        this.getEntry(module, name).flatMap( e => Some(this.eval(this.resolve(e.v))) )

    def getProcessedValueOrPanic(module: Sym, name: Term): Term =
        this.getEntry(module, name) match {
            case Some(e) => this.eval(e.v)
            case None => throw new NoSuchElementException(s"Module $module does not have an entry named $name")
        }

    def getModuleEntries(m: Sym): List[Entry] = this.entList.getOrElse(m, Nil)

    def resolve( t: Term ): Term = t match {
        case Tuple(args @ _*) => Tuple(args.map(x => this.resolve(x)): _*)
        case ListTerm(list) => ListTerm(list.map(x => this.resolve(x)))
        case SetTerm(set) => SetTerm(set.map(x => this.resolve(x)))
        case KeyVal(key, value) => KeyVal(this.resolve(key), this.resolve(value))
        case r @ EntRef(_, _, _) => resolve(resolveReference(r))
        case _ => t
    }

    def resolveReference( r: EntRef ): Term = {
        resolveReferenceOnce(r) match {
            case Some(next @ EntRef(_, _, _)) if next == r => next
            case None => throw new IllegalArgumentException(s"Cannot resolve reference: $r target=${r.mod} (class: ${r.getClass.getSimpleName})" )
            case Some(next @ EntRef(_, _, _)) => resolveReference(next)
            case Some(v) => v
        }
    }

    private def resolveReferenceOnce( r: EntRef ): Option[Term] =
        r match {
            case EntRef(_, t @ EntRef(_, _, _), _) => Some(t)
            case EntRef(m, t, a) => this.getEntry(this.findModuleAlias(m, a), t) match { 
                case Some(Entry(Sym("#def"), _, _)) => Some(r)
                case Some(Entry(_, _, v)) => Some(v)
                case _=> None
            }
        }

    def addModuleAlias(module: Sym, alias: Sym, target: Sym) = {
        this.aliasReg((module, alias)) = target
    }

    def findModuleAlias(source: Sym, alias: Sym):Sym = {
        this.aliasReg.get((source, alias)) match {
            case Some(uri) => uri
            case None => throw new IllegalArgumentException(s"Alias $alias not defined in module $source.\nAliases are added with requirement entries such as: (#req $alias MODULE), where MODULE is the symbolic name of a module or a relative filename.")

        }
    }
    def findSelfAlias(source: Sym):Sym =
        this.aliasReg.find((k, target) => k(0) == target).get(0)(1)

    def typeCheckAllModules(verbose: Boolean = false): Boolean = {
        modList.forall(m => {
            println(s"Module: $m")
            typeCheckModule(m)
        })
    }

    private def checkSingleType( m: Sym, t: Term, v: Term ): Boolean =
        eval(t) match {
            case fr: FunRef => t(this.eval(resolve(v))).asBool.v
            case s: Sym if (this.specialTypes contains s) => true
            case furi: Sym => this.getFunctionOrPanic(furi)(this.eval(resolve(v))).asBool.v
            case _ => {
                val hint = if (!t.isInstanceOf[EntRef]) "" else s"\nHint: If this entry reference corresponds to a type, try using ^$t instead (adding the ^) to make it a function reference."
                throw new IllegalArgumentException(s"Bad type specifier ${t} in module $m. Use symbolic type URI or function reference instead.$hint")
            }
        }


    def typeCheckModule(uri: Sym, verbose: Boolean = false): Boolean = {
        entList.get(uri) match {
            case Some(es) => {
                es.forall( e => {
                    val isConsistent = try {
                        checkSingleType(uri, e.t, e.v)
                    } catch {
                        case ex => {
                            System.err.println(s"Error when checking type ${e.t} in module $uri with value ${e.v}. Turning on verbose and running again.")
                            ex.printStackTrace()
                            this.funReg.values.foreach( f => {
                                if (f.isInstanceOf[Verbose])
                                    f.asInstanceOf[Verbose].setVerbose(1)
                            } )
                            checkSingleType(uri, e.t, e.v)
                            System.exit(1)
                            false
                        }
                    }
                    if ( verbose && !(this.specialTypes.contains(e.t) ) ) println(s"$uri | ${e.t} | ${e.n} | $isConsistent ")

                    if ( verbose && !isConsistent ) {
                        this.funReg.values.foreach( f => {
                            if (f.isInstanceOf[Verbose])
                                f.asInstanceOf[Verbose].setVerbose(1)
                        } )
                        checkSingleType(uri, e.t, e.v)
                    }

                    isConsistent
                })
            }
            case None => {
                System.err.println("Known modules:")
                modList.foreach(System.err.println)
                throw new IllegalArgumentException(s"Module $uri does not exist.")
            }
        }
    }


    override def toString(): String = this.modList.map( m => this.entList.get(m).get.reverse.mkString("", "\n", "") ).toList.reverse.mkString("", "\n", "")
}

