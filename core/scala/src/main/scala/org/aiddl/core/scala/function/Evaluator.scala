package org.aiddl.core.scala.function

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Function, LazyFunction, Verbose, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger

class Evaluator( c: Container ) extends Function with Verbose {
  var followRefs = false

  private val SELF = Sym("#self")
  private val SELF_ALT = Sym("#arg")

  /**
   * Evaluate a term while following all references.
   *
   * This is equivalent to turning on followRefs, applying the evaluator, and turning it off again.
   *
   * @param x term to evaluate
   * @return evaluated term
   */
  def evalAllRefs(x: Term): Term = {
    this.followRefs = true
    val r = this(x)
    this.followRefs = false
    r
  }

  /**
   * Evaluate a term
   *
   * @param x input term
   * @return result of applying the function to <code>x</code>
   */
  override def apply(x: Term): Term = {
    logger.info(x.toString)
    this.logger.depth += 1
    val r = x match {
      case ListTerm(l) => ListTerm(l.map( t => this(t) ))
      case SetTerm(s) => SetTerm(s.map( t => this(t) ))
      case KeyVal(k, v) => KeyVal(this(k), this(v))
      case r @ EntRef(_, _, _) =>
        if (followRefs) this(c.resolveReference(r)) else c.resolveReference(r)
      case tuple @ Tuple(op, args @ _*) =>
        val uri = op match {
          case uri @ Sym(_) if c.hasFunction(uri) => Some(uri)
          case r @ EntRef(mod, name, alias) =>
            val uri = name match {
              case name@Sym(_) =>
                val testUri = c.findModuleAlias(mod, alias) + name
                if c.hasFunction(testUri)
                then testUri
                else this.apply(c.resolveReference(r))
              case _ => this.apply(c.resolveReference(r))
            }
            uri match {
              case uri@Sym(_) => if (c.hasFunction(uri)) Some(uri) else None
              case _ => None
            }
          case _ => this.apply(op) match {
            case uri @ Sym(_) if c.hasFunction(uri) => Some(uri)
            case _ => None
          }
        }
        uri match {
          case Some(uri) =>
            val f = c.getFunctionOrPanic(uri)
            val resolvedArgs =
              if (f.isInstanceOf[LazyFunction]) Tuple(args: _*)
              else
                for {
                  i <- args.indices
                  subbedArg =
                    if (uri != D.TYPE || i == 1) {
                      args(i) match {
                        case e: EntRef => c.resolveReference(e)
                        case x => x
                      }
                    } else {
                      args(i)
                    }
                  resolvedArg = this(subbedArg)
                } yield resolvedArg
            val arg = if ( resolvedArgs.size == 1 ) resolvedArgs.head else Tuple(resolvedArgs: _*)
            logger.fine(s"Applying $uri to $arg")
            f(arg)
          case None =>
            logger.fine(s"Not a function: $op")
            Tuple(tuple.map(t => this(t) ): _*)
        }
      case _ => x
    }
    logger.info(s"Result: $r")
    this.logger.depth -= 1
    r
  }

}
