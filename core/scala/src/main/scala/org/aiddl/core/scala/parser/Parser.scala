package org.aiddl.core.scala.parser

import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.`type`.{GenericTypeChecker, TypeFunction}
import org.aiddl.core.scala.function.misc.NamedFunction
import org.aiddl.core.scala.function.{Evaluator, Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.parser.Parser.*
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.{FilenameResolver, StopWatch}

import java.io.{File, FileNotFoundException}
import scala.annotation.tailrec
import scala.collection.immutable.ArraySeq
import scala.collection.mutable
import scala.collection.mutable.HashMap
import scala.io.{BufferedSource, Source}
import scala.language.postfixOps

object Parser {
    protected[parser] val IntRegEx = """0|(?:\+|-)?[1-9][0-9]*""".r
    protected[parser] val BinRegEx = """#b((?:\+|-)?[01]+)""".r
    protected[parser] val OctRegEx = """#o((?:\+|-)?[0-7]+)""".r
    protected[parser] val HexRegEx = """#x((?:\+|-)?[0-9a-fA-F]+)""".r
    protected[parser] val SciNotRealRegEx = """(?:\+|-)?(?:0|[1-9][0-9]*)\.[0-9]+[eE](?:0|(?:\+|-)?[1-9][0-9]*)""".r
    protected[parser] val SymRegExStr = """[#a-zA-Z\*/\.=\+-][a-zA-Z\+\*-/\.=_0-9\?\#]*"""
    protected[parser] val SymRegEx = SymRegExStr.r
    protected[parser] val VarRegEx = ("""\?""" + SymRegEx).r
    protected[parser] val RationalRegEx = """(0|[\+-]?[1-9][0-9]*)/([1-9][0-9]*)""".r
    protected[parser] val RealRegEx = """(?:\+|-)?(?:0|[1-9][0-9]*)\.[0-9]+""".r
    protected[parser] val InfRegEx = """[\+-]?INF""".r
    protected[parser] val SpecialRegEx = """([\{\}\(\)\[\]\@\^\:\$])"""
    protected[parser] val WhiteRegEx = """[ \n\t\r,]+"""
    protected[parser] val StrRegEx = """"[^"\\]*(?:\\.[^"\\]*)*"""".r
    protected[parser] val CommentRegEx = """;[^\n]*\n"""
    protected[parser] val Special: Set[Term] = Set(Sym("("), Sym("["), Sym("{"), Sym(":"), Sym("$"), Sym("@"), Sym("^"))
    protected[parser] val SpecialTypes: Set[Sym] = Set(Sym("#mod"), Sym("#req"), Sym("#nms"), Sym("#namespace"), Sym("#def"), Sym("#type"), Sym("#interface"), Sym("#assert") )

    protected[scala] def module2filename(module: Sym): Option[String] = {
        val filename = s"aiddl/${module.toString.replace(".", "/")}.aiddl"
        Some(filename)
    }
}

/**
 * Instance of an AIDDL parser
 * @param container container used by the parser
 */
class Parser(container: Container) {
    private val parsedFiles: mutable.HashMap[String, Sym] = new HashMap
    private var aiddlPaths: List[String] = getPathListFromEnv
    private var aiddlClassLoaders: List[ClassLoader] = List(Parser.getClass.getClassLoader)

    /**
     * Parse a string into a single term.
     * @param str string to parse
     * @return term parsed from string
     */
    def str( str: String ): Term =
        this.parse(str).head

    /**
     * Parse a file as a module into the parser's container.
     * @param filename name of the file containing the AIDDL module
     * @return symbolic name of the parsed module
     */
    def parseFile( filename: String ): Sym = {
        this.parseInto(filename, parsedFiles)
    }

    /**
     * Add a path used when searching for AIDDL files and modules
     * @param pathStr the path to add
     */
    def addAiddlPath(pathStr: String): Unit =
        this.aiddlPaths = pathStr :: this.aiddlPaths

    /**
     * Add a class loader that can be used to load AIDDL resources shipped with dependency libraries
     * @param cl class loader to add
     */
    def addClassLoader(cl: ClassLoader): Unit =
        this.aiddlClassLoaders = cl :: this.aiddlClassLoaders

    private def getPathListFromEnv: List[String] = {
        val aiddlPath = scala.util.Properties.envOrElse("AIDDL_PATH", "undefined")
        if aiddlPath.contains(";")
        then aiddlPath.split(";").toList
        else aiddlPath.split(":").toList
    }

    private def getModuleFilename(t: Term, currentFile: String): Option[String] = t match {
        case uri@Sym(_) =>
            Parser.module2filename(uri)
        case Str(filename) =>
            Some(s"${new File(currentFile).getParentFile.getPath}/$filename")
        case o =>
            Some(FilenameResolver(o).toString)
    }

    private def parse(str: String): List[Term] = {
        var s = str
        val sub = new Substitution()

        // Replace strings with symbolic code for later substitution
        var strId = 1
        val strings = StrRegEx.findAllIn(s)
        val strMap = mutable.HashMap[String, String]()

        strings.foreach(x => {
            strId += 1; sub.add(Sym("§" + strId), Str(x.substring(1, x.length() - 1))); s = s.replace(x, "§" + strId)
        })
        // Create token list
        val tokens = s.replaceAll(SpecialRegEx, " $1 ").trim.split(WhiteRegEx).filter(x => !(x isBlank)).toList

        // Parse tokens into terms
        processToken(tokens, Nil, container).map(_ \ sub).reverse
    }

    private def tryGetFileBufferedSourceLocal(filename: String): Option[BufferedSource] = {
        try {
            Some(Source.fromFile(filename))
        } catch {
            case _: FileNotFoundException => None
        }
    }

    private def tryGetFileBufferedSourceResource(filename: String, loader: ClassLoader): Option[BufferedSource] = {
        try {
            Some(Source.fromResource(filename, loader))
        } catch {
            case _: FileNotFoundException => None
        }
    }

    private def parseInto(filename: String, parsedFiles: mutable.HashMap[String, Sym]): Sym = {
        if (parsedFiles contains filename) {
            parsedFiles(filename)
        } else {
            if (!this.container.hasFunction(D.EVAL)) Function.loadDefaultFunctions(this.container)
            val eval = this.container.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

            var fileBufferedSource = tryGetFileBufferedSourceLocal(filename)
            if (fileBufferedSource.isEmpty) {
                aiddlClassLoaders.foreach(loader => {
                    if (fileBufferedSource.isEmpty) {
                        fileBufferedSource = tryGetFileBufferedSourceResource(filename, loader)
                    }
                })
            }
            if (fileBufferedSource.isEmpty) {
                aiddlPaths.foreach(path => {
                    if (fileBufferedSource.isEmpty) {
                        fileBufferedSource = tryGetFileBufferedSourceLocal(s"$path/$filename")
                    }
                })
            }
            if (fileBufferedSource.isEmpty) {
                throw IllegalArgumentException(s"File $filename not found in possible locations:\n\t(1) locally,\n\t(2) AIDDL_PATH: ${aiddlPaths.mkString(", ")}\n\t(3) Resources of registered class loaders: ${aiddlClassLoaders.mkString(", ")}")
            }

            //getFileBufferedSource(filename)

            val fileContents = fileBufferedSource.get.mkString.replaceAll(CommentRegEx, "\n")
            val terms = this.parse(fileContents)

            // Extract module info from first entry
            val modTerm = terms.head
            val selfReference = modTerm.asTup(1).asSym
            val moduleUri = modTerm.asTup(2).asSym
            parsedFiles.put(filename, moduleUri)
            //parsedModuleFilenameMap.put(moduleUri, filename)

            val prefixMap = new mutable.HashMap[String, String]
            prefixMap.put("§self", moduleUri.asSym.name)

            var sub = new Substitution()
            sub.add(Sym("§self"), selfReference)
            sub.add(Sym("§module"), moduleUri)

            this.container.addModuleAlias(moduleUri, selfReference, moduleUri)

            // Add all entries to container
            terms.foreach(x => {
                substituteFunRefPrefix(x \ sub, prefixMap, moduleUri) match {
                    case entry@Tuple(typeTerm, name, value) =>
                        val typeFunRef = typeTerm match {
                            case fr: FunRef => fr
                            case s: Sym if SpecialTypes contains s => s
                            case s: Sym => FunRef.create(s, x => this.container.getFunctionOrPanic(x))
                            case t => this.container.eval(t)
                        }
                        this.container.setEntry(moduleUri, Entry(typeFunRef, name, value))

                        if (typeTerm == Sym("#req")) {
                            this.processRequirement(moduleUri, name, value, filename, prefixMap)
                        } else if (typeTerm == Sym("#nms") || typeTerm == Sym("#namespace")) {
                            sub = this.processNamespace(moduleUri, typeFunRef, name, value, x, filename, sub)
                        } else if (typeTerm == Sym("#def")) {
                            this.processFunctionDefinition(moduleUri, name, value, filename)
                        } else if (typeTerm == Sym("#type")) {
                            this.processTypeDefinition(moduleUri, name, value)
                        } else if (typeTerm == Sym("#interface")) {
                            this.processInterfaceDefinition(moduleUri, name, value)
                        }
                    case _ =>
                        throw new IllegalArgumentException(s"Parsing failed in module: $moduleUri\nTerm is not an entry: $x")
                }
            })
            moduleUri
        }
    }

    private def processRequirement(module: Sym, name: Term, value: Term, filename: String, prefixMap: mutable.HashMap[String, String]): Unit = {
        getModuleFilename(value, filename) match {
            case Some(absoluteFilename) =>
                val reqMod = parseInto(absoluteFilename, parsedFiles)
                prefixMap.put(s"§$name", reqMod.name)
                this.container.addModuleAlias(module, name.asSym, reqMod)
            case None =>
                throw new IllegalArgumentException(s"$filename: Unknown module: " + value + " (type: " + value.getClass.getSimpleName + " use relative filename or uri found in AIDDL_PATH environment variable)")
        }
    }

    private def processNamespace(module: Sym, typeRef: Term, name: Term, value: Term, original: Term, filename: String, sub: Substitution): Substitution = {
        value match {
            case s: Sym =>
                println(s"[Warning] Deprecated #namespace/#nms usage: $s in file $filename")
                getModuleFilename(value, filename) match {
                    case Some(absoluteFilename) =>
                        val nmsMod = parseInto(absoluteFilename, parsedFiles)
                        this.container.getModuleEntries(nmsMod).foreach {
                            case Entry(t, n, v) if t != Sym("#mod") => sub.add(n, v)
                            case _ =>
                        }
                    case None =>
                        throw new IllegalArgumentException(filename + ": Unknown module: " + value + " (type: " + value.getClass.getSimpleName + " use relative filename or uri found in AIDDL_PATH environment variable)")
                }
                sub
            case _ =>
                if (value.isInstanceOf[EntRef]) {
                    if (value.asEntRef.name == Sym("hashtag")) {
                        println(s"[Warning] Namespace hashtag has been deprecated ($filename)")
                    }
                }
                val namespaceSub =
                    try {
                        val sub = Substitution.from(this.container.resolve(original(2)).asCol)
                        this.container.setEntry(module, Entry(typeRef, name, original(2)))
                        sub
                    } catch {
                        case _: Throwable => Substitution.from(this.container.resolve(value).asCol)
                    }
                {
                    sub + namespaceSub
                } match {
                    case Some(newSub) => newSub
                    case None =>
                        throw new IllegalArgumentException(s"Namespace entry $original leads to incompatibility.")
                }
        }
    }

    private def processFunctionDefinition(module: Sym, name: Term, value: Term, filename: String): Unit =
        val f_cfg = name match {
            case Sym(_) => (module + name.asSym, None)
            case Tuple(uri@Sym(_), args) => (module + uri, Some(args))
            case _ =>
                throw new IllegalArgumentException(s"$filename: #def entry name must be symbolic or tuple with symbolic first element. Found:\n$name")
        }
        this.container.addFunction(f_cfg._1, new NamedFunction(value, this.container.eval, f_cfg._2))

    private def processTypeDefinition(module: Sym, name: Term, value: Term): Unit = {
        name match {
            case name: Sym =>
                try {
                    this.container.eval.followRefs = true
                    val typeDef = this.container.eval(value)
                    this.container.eval.followRefs = false
                    val fun = new TypeFunction(typeDef, this.container.eval)
                    this.container.addFunction(module + name, fun)
                } catch {
                    case ex: Exception =>
                        throw new IllegalArgumentException(s"Exception when evaluating #type expression while parsing module: $module\n\tName: $name\n\t$value\n${ex.toString}")
                    case ex: Error =>
                        throw new IllegalArgumentException(s"Exception when evaluating #type expression while parsing module: $module\n\tName: $name\n\t$value\n${ex.toString}")
                }
            case term: Tuple =>
                val baseUri = module + name(0).asSym
                this.container.eval.followRefs = true
                val typeDef = this.container.eval(value)
                this.container.eval.followRefs = false
                val genArgs = if (term.length == 2) term(1) else Tuple(term.tail: _*)
                val fun = new GenericTypeChecker(baseUri, genArgs, typeDef, this.container.eval, this.container)
                this.container.addFunction(baseUri, fun)
            case _ =>
                throw IllegalArgumentException(s"Unsupported #type definition with name $name and value $value")
        }
    }

    private def processInterfaceDefinition(module: Sym, name: Term, value: Term): Unit = {
        name match
            case name: Sym =>
                val uri = this.container.eval(value.getOrElse(Sym("uri"), module + name)).asSym
                this.container.addInterfaceDef(uri, value)
            case _ =>
                throw IllegalArgumentException(s"#interface cannot have non-symbolic name: $name")
    }


    @tailrec
    private def lookBack(stack: List[Term], c: Container): List[Term] = {
        val newStack = if (stack != Nil && !Special.contains(stack.head)) stack match {
            case value :: Sym(":") :: KeyVal(k1, v1) :: xs => KeyVal(k1, KeyVal(v1, value)) :: xs
            case value :: Sym(":") :: key :: xs => KeyVal(key, value) :: xs
            case alias :: Sym("@") :: name :: xs =>
                (name match {
                    case _: FunRef =>
                        FunRef.create(Sym(s"§$alias") + name.asFunRef.uri, x => c.getFunctionOrPanic(x))
                    case _: KeyVal =>
                        KeyVal(name.asKvp.key, EntRef(Sym("§module"), name.asKvp.value, alias.asSym))
                    case _ =>
                        EntRef(Sym("§module"), name, alias.asSym)
                }) :: stack.drop(3)
            case name :: Sym("$") :: xs =>
                EntRef(Sym("§module"), name, Sym("§self")) :: xs
            case ref :: Sym("^") :: xs =>
                ref match {
                    case uri@Sym(_) => FunRef.create(uri, x =>
                        c.getFunctionOrPanic(x)) :: xs
                    case EntRef(mod, n@Sym(name), alias) =>
                        FunRef.create(Sym(s"§${alias.toString.replaceFirst("§", "")}") + n, x => c.getFunctionOrPanic(x)) :: xs
                    case _ =>
                        throw new IllegalArgumentException("Function reference must be symbolic or entry reference.")
                }
            case _ => stack
        } else {
            stack
        }
        if (newStack != stack) {
            lookBack(newStack, c)
        } else {
            stack
        }
    }

    @tailrec
    protected[parser] final def processToken(tokens: List[String], stack: List[Term], c: Container): List[Term] =
        tokens match {
            case Nil => stack
            case x :: xs =>
                var newStack = x match {
                    case IntRegEx() => Num(x toLong) :: stack
                    case BinRegEx(bStr) => Num(java.lang.Integer.parseInt(bStr, 2)) :: stack
                    case OctRegEx(oStr) => Num(java.lang.Integer.parseInt(oStr, 8)) :: stack
                    case HexRegEx(xStr) => Num(java.lang.Integer.parseInt(xStr, 16)) :: stack
                    case RationalRegEx(n, d) => Num(n toLong, d toLong) :: stack
                    case RealRegEx() => Real(x toDouble) :: stack
                    case SciNotRealRegEx() => Real(x.toDouble) :: stack
                    case InfRegEx() => (if (x.contains('-')) {
                        InfNeg()
                    } else {
                        InfPos()
                    }) :: stack
                    case "true" => Bool(true) :: stack
                    case "false" => Bool(false) :: stack
                    case "NaN" => NaN() :: stack
                    case SymRegEx() => Sym(x) :: stack
                    case VarRegEx() => Var(x substring 1) :: stack
                    case StrRegEx() => Str(x) :: stack
                    case "_" => Var() :: stack
                    case ")" => stack.span(_ != Sym("(")) match {
                        case (args, newStack) => Tuple(args.reverse: _*) :: {
                            if (newStack == Nil) {
                                Nil
                            } else {
                                newStack.tail
                            }
                        }
                    }
                    case "}" => stack.span(_ != Sym("{")) match {
                        case (args, newStack) => SetTerm(args.reverse toSet) :: {
                            if (newStack == Nil) {
                                Nil
                            } else {
                                newStack.tail
                            }
                        }
                    }
                    case "]" => stack.span(_ != Sym("[")) match {
                        case (args, newStack) => ListTerm(args.reverse.toVector) :: {
                            if (newStack == Nil) {
                                Nil
                            } else {
                                newStack.tail
                            }
                        }
                    }
                    case _ => Sym(x) :: stack
                }
                newStack =
                    if tokens.tail != Nil && tokens.tail.head == "@"
                    then newStack
                    else lookBack(newStack, c)
                processToken(tokens.tail, newStack, c)

        }

    private def substituteFunRefPrefix(term: Term, prefixMap: mutable.HashMap[String, String], module: Sym): Term =
        if (term.isInstanceOf[FunRef] && term.asFunRef.uri.name.startsWith("§")) {
            val uriStr = term.asFunRef.uri.name
            val sRepl = uriStr.takeWhile(c => c != '.')
            if !prefixMap.contains(sRepl)
            then throw new IllegalStateException(s"Module reference ${sRepl.substring(1)} not loaded $module. This probably means a (#req ${sRepl.substring(1)} some.uri) entry is needed before this reference is used.")
            else FunRef.create(Sym(uriStr.replace(sRepl, prefixMap(sRepl))), x => this.container.getFunctionOrPanic(x))
        } else term match {
            case ListTerm(list) =>
                ListTerm(list.map(x => substituteFunRefPrefix(x, prefixMap, module)))
            case SetTerm(set) =>
                SetTerm(set.map(x => substituteFunRefPrefix(x, prefixMap, module)))
            case Tuple(tuple*) =>
                Tuple(tuple.map(x => substituteFunRefPrefix(x, prefixMap, module)): _*)
            case KeyVal(key, value) =>
                KeyVal(substituteFunRefPrefix(key, prefixMap, module), substituteFunRefPrefix(value, prefixMap, module))
            case _ =>
                term
        }
}