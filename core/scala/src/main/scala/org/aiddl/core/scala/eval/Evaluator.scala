package org.aiddl.core.scala.eval

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Function, LazyFunction, Verbose, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.tools.Logger

class Evaluator( c: Container ) extends Function with Verbose {
  var followRefs = false

  val SELF = Sym("#self")
  val SELF_ALT = Sym("#arg")

  def evalAllRefs(x: Term): Term = {
    this.followRefs = true
    val r = this(x)
    this.followRefs = false
    r
  }

    override def apply(x: Term): Term = this(x, Nil)

    private def evaluatable( x: Term ): Boolean = x match {
        case EntRef(_, _, _) => true
        case KeyVal(k, v) => evaluatable(k) || evaluatable(v)
        case ListTerm(l) => l.exists( e => evaluatable(e) )
        case SetTerm(l) => l.exists( e => evaluatable(e) )
        case Tuple(op @ Sym(_), _*) if c.hasFunction(op) => true
        case Tuple(args @ _*) => args.exists( evaluatable(_) )
        case _=> false
    }

    def apply(x: Term, selfStack: List[Substitution]): Term = {
        if ( !evaluatable(x) ) x else {
            logInc(1, x.toString())
            val r = x match {
                case ListTerm(l) => ListTerm(l.map( t => this(t) ))
                case SetTerm(s) => SetTerm(s.map( t => this(t) ))
                case KeyVal(k, v) => KeyVal(this(k), this(v))
                case r @ EntRef(mod, name, alias) => {
                    if (followRefs) this(c.resolveReference(r)) else c.resolveReference(r)
                }
                case tuple @ Tuple(op, args @ _*) => {
                    val uri = op match { // get Some(uri) if uri is a known function
                        case uri @ Sym(_) if c.hasFunction(uri) => Some(uri)
                        case r @ EntRef(mod, name, alias) => {
                            val uri = name match {
                                case name@Sym(_) => c.findModuleAlias(mod, alias) + name
                                case _ => this (c.resolveReference(r))
                            }
                            uri match {
                                case uri@Sym(_) => if (c.hasFunction(uri.asSym)) Some(uri) else None
                                case _ => None
                            }
                        }
                        case _ => this(op) match {
                            case uri @ Sym(_) if c.hasFunction(uri) => Some(uri)
                            case _ => None
                        }
                    }
                    uri match {
                        case Some(uri) => {
                            val f = c.getFunctionOrPanic(uri)
                            var newStack = selfStack
                            var resolvedArgs = if (f.isInstanceOf[LazyFunction]) Tuple(args: _*) else
                                for {
                                    i <- 0 until args.length
                                    subbedArg =
                                      if (uri != D.TYPE || i == 1)
                                          if ( selfStack == Nil ) (args(i) match {
                                              case e: EntRef => c.resolveReference(e)
                                              case x => x })
                                          else (args(i) match {
                                              case e: EntRef => c.resolveReference(e)
                                              case x => x })\selfStack.head
                                      else if ( uri == D.TYPE && i == 2 ) {
                                          val selfValue = this(args(0), selfStack)
                                          val s = new Substitution()
                                          s.add( SELF, selfValue )
                                          s.add( SELF_ALT, selfValue )
                                          (args(i) match { case e: EntRef => c.resolveReference(e) case x => x }) \s
                                      } else args(i)
                                    resolvedArg = this(subbedArg, newStack)
                                } yield resolvedArg
                            val arg = if ( resolvedArgs.size == 1 ) resolvedArgs(0) else Tuple(resolvedArgs: _*)
                            log(2, "Applying " + uri + " to " + arg)
                            f(arg)
                        }
                        case None => { log(2, "Not a function: " + op); Tuple(tuple.map( t => this(t, selfStack) ): _*) }
                    }
                }
                case _ => x
            }
            logDec(1, "Result: " + r.toString())
            r
        }
    }
}
