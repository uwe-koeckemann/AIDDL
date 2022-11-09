package org.aiddl.external.scala.prolog

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.util.FilenameResolver
import org.aiddl.external.scala.prolog.tools.ExecuteSystemCommand

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths, StandardOpenOption}
import scala.collection.mutable
import scala.io.Source

class QuerySolver extends Function with Verbose {
  var workDir = System.getProperty("java.io.tmpdir")

  val prolog2term: mutable.HashMap[String, Term] = new mutable.HashMap[String, Term]()

  def apply( x: Term ): Term = {
    val hash = x.hashCode()
    val query = x(0)
    val kbTerm = x(1).asCol

    val kbBuilder = new StringBuilder()
    kbTerm.foreach(entry => {
      entry match {
        case c: CollectionTerm => {
          val sb = new StringBuilder()
          c.foreach(a => {
            val prologStr = this.atom2prolog(a)
            sb.append(prologStr)
            sb.append(".\n")
          })
          kbBuilder.append(sb.toString())
        }
        case fs => {
          val fn = FilenameResolver(fs)
          kbBuilder.append(Source.fromFile(fn.asStr.value).getLines.mkString("\n"))
        }
      }
    })

    val kbFile = s"${this.workDir}/kb_$hash.prolog"
    val answerFile = s"${this.workDir}/answer_$hash.prolog"
    val runFile = s"${this.workDir}/run_$hash.prolog"

    val varBuilder = new StringBuilder()
    val varBagBuilder = new StringBuilder()
    val queryPredBuilder = new StringBuilder()

    var variables: List[Var] = Nil

    {
      var i = 0
      query.asCol.foreach(q => {
        variables = variables ++ getVariables(q)
        val prologStr = atom2prolog(q)
        queryPredBuilder.append(prologStr)
        if (i < query.length - 1)
          queryPredBuilder.append(", ")
        i += 1
      })
      queryPredBuilder.append(".")
      variables = variables.distinct
    }
    {
      var i = 0
      variables.foreach(x => {
        val prologStr = var2prolog(x)
        prolog2term.put(prologStr, x)
        varBuilder.append(prologStr)
        varBagBuilder.append(prologStr)
        if (i < variables.length - 1) {
          varBuilder.append(", ")
          varBagBuilder.append("/")
        }
        i += 1
      })
    }
    val varsStr = varBuilder.toString

    val queryPred = if (variables.isEmpty) {
      s"queryPred :- ${queryPredBuilder.toString}"
    } else {
      s"queryPred(${varsStr}) :- ${queryPredBuilder.toString}"
    }
    val bagOf = if (variables.isEmpty) {
      s"start :- ['$kbFile'], queryPred -> tell('$answerFile'), write([]) ; tell('$answerFile'), write('[-]')."
    } else {
      s"start :- ['$kbFile'], " +
        s"bagof( $varBagBuilder, queryPred($varsStr),QueryResult ) -> tell('$answerFile'), " +
        s"write(QueryResult) ; " +
        s"tell('$answerFile'), " +
        s"write('[-]')."
    }

    kbBuilder.append(queryPred.toString)

    this.dump2file(kbBuilder.toString(), kbFile)
    this.dump2file(bagOf.toString(), runFile)

    val cmd = s"swipl -s $runFile -g start -t halt"
    logger.info(s"Query: $queryPred")
    logger.info(s"Command: $cmd")
    val conOut = ExecuteSystemCommand.call(this.workDir, cmd)

    conOut match {
      case Some(sout, serr) => {
        println("==================================")
        println(s"STANDARD OUT for $cmd")
        println("==================================")
        println(sout)
        println("==================================")
        println(s"STANDARD ERR: for $cmd")
        println("==================================")
        println(serr)
      }
      case None => {}
    }

    val r = this.getResults(answerFile, variables, prolog2term)

    r match {
      case None => {
        logger.info("No results")
        Common.NIL
      }
      case Some(rs) => {
        val results = ListTerm(rs.map( s => {
          ListTerm(variables.map( x => KeyVal(x, s.get(x)) ).toVector)
        }).distinct.toVector)
        logger.info(s"Query results: $results")
        results
      }
    }
  }

  def dump2file( s: String, filename: String ) = {
    Files.write(
      Paths.get(filename),
      s.getBytes(StandardCharsets.US_ASCII),
      StandardOpenOption.CREATE,
      StandardOpenOption.WRITE,
      StandardOpenOption.TRUNCATE_EXISTING
    )
  }

  def getVariables( t: Term ): List[Var] = {
    t match {
      case x: Var => List(x)
      case c: CollectionTerm => c.flatMap(e => getVariables(e)).toList
      case c: Tuple => c.x.flatMap(e => getVariables(e)).toList
      case _ => Nil
    }
  }

  def var2prolog( x: Var ): String = {
    if ( x.name.charAt(0) == '_' ) "_"
    else s"Var_${x.name}"
  }

  def atom2prolog( a: Term ): String = {
    val sb = new StringBuilder()

    val name = a(0).toString.toLowerCase
    this.prolog2term.put(name, a(0))
    sb.append(name)
    sb.append("(")

    (1 until a.length).foreach( i => {
      val arg = a(i)
      val s = arg match {
        case Sym(name) => s"'$name'"
        case x: Var => var2prolog(x)
        case s: Tuple => atom2prolog(s)
        case _ => throw new IllegalArgumentException(s"Unsupported argument term $arg for atom $a")
      }
      prolog2term.put(s, arg)
      sb.append(s)
      if ( i < a.length-1 )
        sb.append(", ")

    })
    sb.append(")")
    sb.toString()
  }

  def getResults(answerFilename: String, queryVars: List[Var], prolog2aiddl: mutable.HashMap[String, Term]): Option[Seq[Substitution]] =
    var results: List[Substitution] = Nil
    Source.fromFile(answerFilename).getLines.foreach( line => {
      if ( line.trim == "[]" || line.trim == "[queryPred]" || line.trim == "[queryPred,queryPred]" ) {
        return Some(Nil)
      } else if ( line.trim == "[-]" ) {
        return None
      }
      val allSubs = complexSplit(line.substring(1, line.length-1), ",")
      allSubs.map( s => {
        val constants = s
          .replaceAll("\\[", "list(")
          .replaceAll("\\]", ")")
          .replaceAll("/", "<####>")
          .split("<####>")

        val theta = new Substitution()
        (0 until constants.length).foreach(
          k => {
            val realVal = prolog2aiddl.getOrElse(
              constants(k),
              prolog2aiddl.getOrElse(
                s"'${constants(k)}'",
                Sym(constants(k))))
            val c =
              if ( constants(k).contains("(") )
                convertTermFormat(constants(k))
              else
                constants(k)
            theta.add(queryVars(k), realVal)
          })
        results = theta :: results
      })
    })
    Some(results.reverse)


  def complexSplit(str: String, sep: String): List[String] = {
    var results: List[String] = Nil
    var depth = 0
    var tmp = new StringBuilder()
    (0 until str.length).foreach( i => {
      val c = str.charAt(i).toString
      if ( c == sep && depth == 0 ) {
        results = tmp.toString() :: results
        tmp = new StringBuilder()
      } else if ( c == "(" || c == "[" ) {
        depth += 1
        tmp.append(c)
      } else if ( c == ")" || c == "]" ) {
        depth -= 1
        tmp.append(c)
      } else {
        tmp.append(c)
      }
    })
    results = tmp.toString() :: results
    results.reverse
  }

  def convertTermFormat(in: String): String = {
    var tmp = s" ${in.replaceAll(",", " ")}"
    var lastSpaceAt = 0
    (0 until tmp.length).foreach( i => {
      if ( tmp.charAt(i) == ' ' ) lastSpaceAt = i
      else if ( tmp.charAt(i) == '(' ) {
        tmp = s"${tmp.substring(0, lastSpaceAt)} (${tmp.substring(lastSpaceAt+1, i)} ${tmp.substring(i+1)}"
      }
    })
    tmp.substring(1)
  }
}
