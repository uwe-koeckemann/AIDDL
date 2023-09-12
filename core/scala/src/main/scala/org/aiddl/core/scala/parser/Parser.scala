package org.aiddl.core.scala.parser

import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.`type`.{GenericTypeChecker, TypeFunction}
import org.aiddl.core.scala.function.misc.NamedFunction
import org.aiddl.core.scala.function.{Evaluator, Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.{FilenameResolver, StopWatch}

import java.io.{File, FileNotFoundException}
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

    var aiddlPaths: List[String] = Nil
    var aiddlClassLoaders: List[ClassLoader] = Nil

    protected[scala] def module2filename( module: Sym ): Option[String] =
        parsedModuleFilenameMap.get(module)

    private def getRecursiveListOfFiles( d: File ): Array[File] = {
        val these = d.listFiles
        these ++ these.filter(_ isDirectory).flatMap(getRecursiveListOfFiles)
    }

    private lazy val moduleFileMap: Map[Sym, String] = {
        val aiddlPath = scala.util.Properties.envOrElse("AIDDL_PATH", "undefined")
        val pathList = if ( aiddlPath.contains(";") ) { aiddlPath.split(";").toList } else { aiddlPath.split(":").toList }
        val fileList = pathList.flatMap( x => {
            val f = new File(x)
            getRecursiveListOfFiles(f) filter( x => !x.getName().contains("~") && x.isFile )
        } )

        fileList.flatMap(x => {
            Source.fromFile(x)
              .getLines().find(l => l contains "#mod") match {
                case Some(s) => List(Sym(s.split("""\)""").head.split(" ").last) -> x.getAbsolutePath())
                case None => Nil
            }
        }).toMap
    }

    private val parsedModuleFilenameMap = new mutable.HashMap[Sym, String]()

    private def getModuleFilename(t: Term, currentFile: String, mfMap: Map[Sym, String]): Option[String] = t match {
        case uri@Sym(_) => mfMap.get(uri)
        case Str(reqFname) => Some((new File(currentFile)).getParentFile().getPath + "/" + reqFname)
        case o => Some(FilenameResolver(o).toString)
    }

    protected def parse( str: String, c: Container ): List[Term] = {
        var s = str
        val sub = new Substitution()

        // Replace strings with symbolic code for later substitution
        var strId = 1
        val strings = StrRegEx.findAllIn(s)
        val strMap = HashMap[String, String]()

        strings.foreach( x => { strId += 1; sub.add(Sym("§"+strId), Str(x.substring(1, x.length()-1))); s = s.replace(x, "§"+strId) } )
        // Create token list
        val tokens = s.replaceAll(SpecialRegEx, " $1 ").trim.split(WhiteRegEx).filter(x => !(x isBlank)).toList

        // Parse tokens into terms
        processToken(tokens, Nil, c).map( _ \ sub ).reverse.toList
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

    protected def parseInto(filename: String, container: Container, parsedFiles: HashMap[String, Sym] ): Sym = {
        if ( parsedFiles contains filename ) {
            parsedFiles(filename)
        } else {
            if ( !container.hasFunction(D.EVAL) ) Function.loadDefaultFunctions(container)
            val eval = container.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

            var fileBufferedSource = tryGetFileBufferedSourceLocal(filename)
            if (fileBufferedSource.isEmpty) {
                aiddlPaths.foreach( path => {
                    if (fileBufferedSource.isEmpty) {
                        fileBufferedSource = tryGetFileBufferedSourceLocal(s"$path/$filename")
                    }
                })
            }
            if (fileBufferedSource.isEmpty) {
                aiddlClassLoaders.foreach(loader => {
                    if (fileBufferedSource.isEmpty) {
                        fileBufferedSource = tryGetFileBufferedSourceResource(filename, loader)
                    }
                })
            }
            if (fileBufferedSource.isEmpty) {
                throw IllegalArgumentException(s"File $filename not found locally,\nin ${aiddlPaths.mkString(", ")}\nor in${aiddlClassLoaders.mkString(", ")}")
            }

            //getFileBufferedSource(filename)

            val fileContents = fileBufferedSource.mkString.replaceAll(CommentRegEx, "\n")
            val terms = this.parse(fileContents, container)

            // Extract module info from first entry
            val modTerm = terms.head
            val selfReference = modTerm.asTup(1).asSym
            val moduleUri = modTerm.asTup(2).asSym
            parsedFiles.put(filename, moduleUri)
            parsedModuleFilenameMap.put(moduleUri, filename)

            val prefixMap = new HashMap[String, String]
            prefixMap.put("§self", moduleUri.asSym.name)

            var sub = new Substitution()
            sub.add(Sym("§self"), selfReference)
            sub.add(Sym("§module"), moduleUri)

            container.addModuleAlias(moduleUri, selfReference, moduleUri)

            // Add all entries to container
            terms.foreach( x => {
                fixSymPrefix((x\sub), prefixMap, container, moduleUri) match {
                    case entry@Tuple(typeTerm, name, value) =>
                        val typeFunRef = typeTerm match {
                            case fr: FunRef => fr
                            case s: Sym if SpecialTypes contains s => s
                            case s: Sym => FunRef.create(s, x => container.getFunctionOrPanic(x))
                            case t => container.eval(t)
                        }
                        container.setEntry(moduleUri, Entry(typeFunRef, name, value))

                        if (typeTerm == Sym("#req")) {
                            this.processRequirement(moduleUri, name, value, filename, container, parsedFiles, prefixMap)
                        } else if (typeTerm == Sym("#nms") || typeTerm == Sym("#namespace")) {
                            value match {
                                case s: Sym => {
                                    println(s"[Warning] Deprecated #namespace/#nms usage: $s in file $filename")
                                    getModuleFilename(value, filename, moduleFileMap) match {
                                        case Some(absoluteFilename) => {
                                            val nmsMod = parseInto(absoluteFilename, container, parsedFiles)
                                            container.getModuleEntries(nmsMod).foreach {
                                                case Entry(t, n, v) if t != Sym("#mod") => sub.add(n, v)
                                                case _ => {}
                                            }
                                        }
                                        case None => throw new IllegalArgumentException(filename + ": Unknown module: " + value + " (type: " + value.getClass.getSimpleName + " use relative filename or uri found in AIDDL_PATH environment variable)")
                                    }
                                }
                                case _ => {
                                    if (value.isInstanceOf[EntRef]) {
                                        if ( value.asEntRef.name == Sym("hashtag") ) {
                                            println(s"[Warning] Namespace hashtag has been deprecated ($filename)")
                                        }
                                    }
                                    try {
                                        val namespaceSub =
                                            try {
                                                val sub = Substitution.from(container.resolve(x(2)).asCol)
                                                container.setEntry(moduleUri, Entry(typeFunRef, name, x(2)))
                                                sub
                                            } catch {
                                                case _: Throwable => Substitution.from(container.resolve(value).asCol)
                                            }
                                        {
                                            sub + namespaceSub
                                        } match {
                                            case Some(newSub) => sub = newSub
                                            case None => throw new IllegalArgumentException(s"Namespace entry $x leads to incompatibility.")
                                        }
                                    }
                                }
                            }
                        } else if (typeTerm == Sym("#def")) {
                            val f_cfg = name match {
                                case Sym(_) => (moduleUri + name.asSym, None)
                                case Tuple(uri@Sym(_), args) => (moduleUri + uri, Some(args))
                                //case Tuple(uri@Sym(_), args: _*) => (moduleUri + uri, Some(Tuple(args: _*)))
                                case _ => throw new IllegalArgumentException(filename + ": #def entry name must be symbolic or tuple with symbolic first element. Found:\n" + entry)
                            }
                            container.addFunction(f_cfg._1, new NamedFunction(value, eval, f_cfg._2))
                        } else if (typeTerm == Sym("#type")) {
                            name match {
                                case name: Sym => {
                                    try {
                                        eval.followRefs = true
                                        val typeDef = eval(value)
                                        eval.followRefs = false
                                        val fun = new TypeFunction(typeDef, container.eval)

                                        container.addFunction(moduleUri + name, fun)
                                    } catch {
                                        case ex => throw new IllegalArgumentException(s"Exception when evaluating #type expression while parsing module: $moduleUri\nExpression: $entry\n" + ex.toString)
                                    }
                                }
                                case term: Tuple => {
                                    val baseUri = moduleUri + name(0).asSym
                                    eval.followRefs = true
                                    val typeDef = eval(value)
                                    eval.followRefs = false
                                    val genArgs = if (term.length == 2) term(1) else Tuple(term.tail: _*)
                                    val fun = new GenericTypeChecker(baseUri, genArgs, typeDef, eval, container)
                                    container.addFunction(baseUri, fun)
                                }
                                case _ => throw IllegalArgumentException(s"Unsupported #type definition: $entry")
                            }
                        } else if (typeTerm == Sym("#interface")) {
                            name match
                                case name: Sym => {
                                    val uri = container.eval(value.getOrElse(Sym("uri"), moduleUri + name)).asSym
                                    container.addInterfaceDef(uri, value)
                                }
                                case _ => throw IllegalArgumentException(s"#interface cannot have non-symbolic name: $entry")
                        }
                    case _ => {
                        throw new IllegalArgumentException(s"Parsing failed in module: $moduleUri\nTerm is not an entry: $x")
                    }
                }
            })
            moduleUri
        }
    }

    private def processRequirement(module: Sym, name: Term, value: Term, filename: String, container: Container, parsedFiles: HashMap[String, Sym], prefixMap: HashMap[String, String]): Unit = {
        getModuleFilename(value, filename, moduleFileMap) match {
            case Some(absoluteFilename) => {
                val reqMod = parseInto(absoluteFilename, container, parsedFiles)
                prefixMap.put(s"§$name", reqMod.name)
                container.addModuleAlias(module, name.asSym, reqMod)
            }
            case None => throw new IllegalArgumentException(s"$filename: Unknown module: " + value + " (type: " + value.getClass.getSimpleName + " use relative filename or uri found in AIDDL_PATH environment variable)")
        }
    }

    private def lookBack( stack: List[Term], c: Container ): List[Term] = {
        val newStack = if (stack != Nil && !Special.contains(stack.head)) stack match {
            case value :: Sym(":") :: KeyVal(k1, v1) :: xs => KeyVal(k1, KeyVal(v1, value)) :: xs
            case value :: Sym(":") :: key :: xs => KeyVal(key, value) :: xs
            case alias :: Sym("@") :: name :: xs =>
                (
                    if (name.isInstanceOf[FunRef]) { FunRef.create(Sym(s"§$alias") + name.asFunRef.uri, x => c.getFunctionOrPanic(x)) }
                    else if ( name.isInstanceOf[KeyVal] ) { KeyVal(name.asKvp.key, EntRef(Sym("§module"), name.asKvp.value, alias.asSym)) }
                    else { EntRef(Sym("§module"), name, alias.asSym) }
                ) :: stack.drop(3)
            case name :: Sym("$") :: xs => EntRef(Sym("§module"), name, Sym("§self")) :: xs
            case ref :: Sym("^") :: xs => ref match {
                case uri @ Sym(_) => FunRef.create(uri, x => c.getFunctionOrPanic(x) ) :: xs
                case EntRef(mod, n @ Sym(name), alias) => FunRef.create(Sym(s"§${alias.toString.replaceFirst("§", "")}") + n, x => c.getFunctionOrPanic(x)) :: xs
                case _ => throw new IllegalArgumentException("Function reference must be symbolic or entry reference.")
            }
            case _ => stack
        } else { stack }
        if ( newStack != stack ) { lookBack(newStack, c) } else { stack }
    }

    protected[parser] def processToken( tokens: List[String], stack: List[Term], c: Container ): List[Term] =
        tokens match {
            case Nil => stack
            case x :: xs => {
                var newStack = x match {
                    case IntRegEx() =>  Integer(x toLong) :: stack
                    case BinRegEx(bStr) => Num(java.lang.Integer.parseInt(bStr, 2)) :: stack
                    case OctRegEx(oStr) => Num(java.lang.Integer.parseInt(oStr, 8)) :: stack
                    case HexRegEx(xStr) => Num(java.lang.Integer.parseInt(xStr, 16)) :: stack
                    case RationalRegEx(n, d) => Num(n toLong, d toLong) :: stack
                    case RealRegEx() => Real(x toDouble) :: stack
                    case SciNotRealRegEx() => Real(x.toDouble) :: stack
                    case InfRegEx() => (if (x.contains('-')) { InfNeg() } else { InfPos() }) :: stack
                    case "true" => Bool(true):: stack
                    case "false" => Bool(false):: stack
                    case "NaN" => NaN() :: stack
                    case SymRegEx() => Sym(x) :: stack
                    case VarRegEx() => Var(x substring 1) :: stack
                    case StrRegEx() => Str(x) :: stack
                    case "_" => Var() :: stack
                    case ")" => stack.span( _ != Sym("(") ) match { case (args, newStack) => Tuple(args.reverse: _*) :: {if (newStack == Nil) { Nil } else { newStack.tail }}}
                    case "}" => stack.span( _ != Sym("{") ) match { case (args, newStack) => SetTerm(args.reverse toSet) :: {if (newStack == Nil) { Nil } else { newStack.tail }}}
                    case "]" => stack.span( _ != Sym("[") ) match { case (args, newStack) => ListTerm(args.reverse.toVector) :: {if (newStack == Nil) { Nil } else { newStack.tail }}}
                    case _ => Sym(x) :: stack
                }
                newStack = if (tokens.tail != Nil && tokens.tail.head == "@") newStack else lookBack(newStack , c)
                processToken( tokens.tail, newStack, c)}}

    private def fixSymPrefix( t: Term, s: HashMap[String, String], c: Container, m: Sym ): Term =
        if ( t.isInstanceOf[FunRef] && t.asFunRef.uri.name.startsWith("§") ) {
            val uriStr = t.asFunRef.uri.name
            val sRepl = uriStr.takeWhile( c => c != '.')
            if ( !s.contains(sRepl) ) { throw new IllegalStateException(s"Module reference ${sRepl.substring(1)} not loaded $m. This probably means a (#req ${sRepl.substring(1)} some.uri) entry is needed before this reference is used.") }
            else FunRef.create(Sym(uriStr.replace(sRepl, s(sRepl))), x => c.getFunctionOrPanic(x))
        } else t match {
            case ListTerm(list) => ListTerm(list.map( x => fixSymPrefix(x, s, c, m)))
            case SetTerm(set) => SetTerm(set.map( x => fixSymPrefix(x, s, c, m)))
            case Tuple(tuple @ _* ) => Tuple(tuple.map( x => fixSymPrefix(x, s, c, m) ): _*)
            case KeyVal(key, value) => KeyVal(fixSymPrefix(key, s, c, m), fixSymPrefix(value, s, c, m))
            case _ => t
        }
}

/**
 * Instance of an AIDDL parser
 * @param c container used by the parser
 */
class Parser(c: Container) {
    val parsedFiles: HashMap[String, Sym] = new HashMap

    /**
     * Parse a string into a single term.
     * @param str string to parse
     * @return term parsed from string
     */
    def str( str: String ): Term = Parser.parse(str, this.c).head

    /**
     * Parse a file as a module into the parser's container.
     * @param filename name of the file containing the AIDDL module
     * @return symbolic name of the parsed module
     */
    def parseFile( filename: String ): Sym = {
        Parser.parseInto(filename, c, parsedFiles)
    }
}