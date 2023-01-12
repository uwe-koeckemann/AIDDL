package org.aiddl.core.scala.parser

import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.`type`.{GenericTypeChecker, TypeFunction}
import org.aiddl.core.scala.function.misc.NamedFunction
import org.aiddl.core.scala.function.{Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.{FilenameResolver, StopWatch}

import java.io.File
import scala.collection.immutable.ArraySeq
import scala.collection.mutable
import scala.collection.mutable.HashMap
import scala.io.Source
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

    protected[scala] def module2filename( module: Sym ): Option[String] = parsedModuleFilenameMap.get(module)

    private def getRecursiveListOfFiles( d: File ): Array[File] = {
        val these = d.listFiles
        these ++ these.filter(_ isDirectory).flatMap(getRecursiveListOfFiles)
    }

    private lazy val moduleFileMap: Map[Sym, String] = {
        val aiddlPath = scala.util.Properties.envOrElse("AIDDL_PATH", "undefined")
        val pathList = if ( aiddlPath.contains(";") ) { aiddlPath.split(";").toList } else { aiddlPath.split(":").toList }
        val fileList = pathList.flatMap( x => {
            val f = new File(x)
            getRecursiveListOfFiles(f) filter( x => !x.getName().toString().contains("~") && x.isFile )
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
        case Str(reqFname) => Some((new File(currentFile)).getParentFile().getAbsolutePath() + "/" + reqFname)
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

        // tokens.foreach(println)
        // Parse tokens into terms
        processToken(tokens, Nil, c).map( _ \ sub ).reverse.toList
    }

    protected def parseInto( fname: String, c: Container, parsedFiles: HashMap[String, Sym] ): Sym = {
        if ( parsedFiles contains fname ) {
            parsedFiles(fname)
        } else {
            if ( !c.hasFunction(D.EVAL) ) Function.loadDefaultFunctions(c)
            val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

            var s = Source.fromFile(fname).mkString.replaceAll(CommentRegEx, "\n")

            var sub = new Substitution()
            val terms = this.parse(s, c)

            // Extract module info from first entry
            val modTerm = terms.head
            //println(s"MOD: $modTerm")
            val selfReference = modTerm.asTup(1).asSym
            val moduleUri = modTerm.asTup(2).asSym
            parsedFiles.put(fname, moduleUri)
            parsedModuleFilenameMap.put(moduleUri, fname)

            val prefixMap = new HashMap[String, String]

            prefixMap.put("§self", moduleUri.asSym.name)
            sub.add(Sym("§self"), selfReference)
            sub.add(Sym("§module"), moduleUri)

            c.addModuleAlias(moduleUri, selfReference, moduleUri)

            // Add all entries to container
            terms.foreach( x => {
                    fixSymPrefix((x\sub), prefixMap, c, moduleUri) match {
                        case e@Tuple(t, n, v) => {
                            val tFunRef = t match {
                                case fr: FunRef => fr
                                case s: Sym if SpecialTypes contains s => s
                                case s: Sym => FunRef.create(s, x => c.getFunctionOrPanic(x))
                                case t => c.eval(t)
                            }
                            c.setEntry(moduleUri, Entry(tFunRef, n, v))

                            if (t == Sym("#req")) {
                                getModuleFilename(v, fname, moduleFileMap) match {
                                    case Some(absReqFname) => {
                                        // println(s"Loading #req $absReqFname")
                                        val reqMod = parseInto(absReqFname, c, parsedFiles)
                                        prefixMap.put("§" + n.asSym.name, reqMod.name)
                                        c.addModuleAlias(moduleUri, n.asSym, reqMod)
                                    }
                                    case None => throw new IllegalArgumentException(fname + ": Unknown module: " + v + " (type: " + v.getClass.getSimpleName + " use relative filename or uri found in AIDDL_PATH environment variable)")
                                }
                            } else if (t == Sym("#nms") || t == Sym("#namespace")) {
                                v match {
                                    case s: Sym => {
                                        println(s"[Warning] Deprecated #namespace/#nms usage: $s in file $fname")
                                        getModuleFilename(v, fname, moduleFileMap) match {
                                            case Some(absReqFname) => {
                                                val nmsMod = parseInto(absReqFname, c, parsedFiles)
                                                c.getModuleEntries(nmsMod).foreach(x => x match {
                                                    case Entry(t, n, v) if t != Sym("#mod") => sub.add(n, v)
                                                    case _ => {}
                                                })
                                            }
                                            case None => throw new IllegalArgumentException(fname + ": Unknown module: " + v + " (type: " + v.getClass.getSimpleName + " use relative filename or uri found in AIDDL_PATH environment variable)")
                                        }
                                    }
                                    case t => {
                                        if (t.isInstanceOf[EntRef]) {
                                            if ( t.asEntRef.name == Sym("hashtag") ) {
                                                println(s"[Warning] Namespace hashtag has been deprecated ($fname)")
                                            }
                                        }
                                        try {
                                            val s = Substitution.from(c.resolve(t).asCol)
                                            {
                                                sub + s
                                            } match {
                                                case Some(newSub) => sub = newSub
                                                case None => throw new IllegalArgumentException(s"Namespace entry $e leads to incompatibility.")
                                            }
                                        } catch {
                                            case ex => {
                                                System.err.println(s"Exception when attempting to apply namespace $t\nModule: $moduleUri\nEntry: $e")
                                                ex.printStackTrace()
                                            }
                                        }

                                    }
                                }
                            } else if (t == Sym("#def")) {
                                val f_cfg = n match {
                                    case Sym(_) => (moduleUri + n, None)
                                    case Tuple(uri@Sym(_), args) => (moduleUri + uri, Some(args))
                                    case Tuple(uri@Sym(_), args: _*) => (moduleUri + uri, Some(Tuple(args: _*)))
                                    case _ => throw new IllegalArgumentException(fname + ": #def entry name must be symbolic or tuple with symbolic first element. Found:\n" + e)
                                }
                                c.addFunction(f_cfg._1, new NamedFunction(v, eval, f_cfg._2))
                            } else if (t == Sym("#type")) {
                                n match {
                                    case name: Sym => {
                                        try {
                                            eval.followRefs = true
                                            val typeDef = eval(v)
                                            eval.followRefs = false
                                            val fun = new TypeFunction(typeDef, c.eval)

                                            c.addFunction(moduleUri + name, fun)
                                        } catch {
                                            case ex => {
                                                System.err.println(s"Exception when evaluating #type expression while parsing module: $moduleUri\nExpression: $e\n")
                                                ex.printStackTrace()
                                            }
                                        }
                                    }
                                    case t: Tuple => {
                                        val baseUri = moduleUri + n(0)
                                        eval.followRefs = true
                                        val typeDef = eval(v)
                                        eval.followRefs = false
                                        val genArgs = if (t.length == 2) t(1) else Tuple(t.tail: _*)
                                        val fun = new GenericTypeChecker(baseUri, genArgs, typeDef, eval, c)
                                        c.addFunction(baseUri, fun)
                                    }
                                    case _ => throw IllegalArgumentException(s"Unsupported #type definition: $e")
                                }
                            } else if (t == Sym("#interface")) {
                                n match
                                    case name: Sym => {
                                        val uri = c.eval(v.getOrElse(Sym("uri"), moduleUri + name)).asSym
                                        c.addInterfaceDef(uri, v)
                                    }
                                    case _ => throw IllegalArgumentException(s"#interface cannot have non-symbolic name: $e")
                            }
                        }
                        case _ => {
                            throw new IllegalArgumentException(s"Parsing failed in module: $moduleUri\nTerm is not an entry: $x")
                        }
                    }
                })

            moduleUri
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