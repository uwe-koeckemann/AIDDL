package org.aiddl.core.scala.tools

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*

import java.io.File

/**
 * Convert relative or absolute filename descriptors to absolute paths in OS format.
 */
object FilenameResolver extends Function {
  val Rel = Sym("relative")
  val Abs = Sym("absolute")

  /**
   * Interpret a term as a filename descriptor with the following format:
   *
   * - (relative "file") relative to program folder
   * - (relative mod "file") relative to a module
   * - (absolute "file") is an absolute path
   * - "file" relative to program or absolute path
   *
   * @param x input term
   *  @return result of applying the function to <code>x</code>
   */
  def apply( x: Term ): Term = x match {
    case Tuple(Rel, module: Sym, Str(filename)) => {
      if (filename.startsWith(".")) {
        val absPath = Parser.module2filename(module).map(s => {
          new java.io.File(new java.io.File(s).getParent).getCanonicalPath
            + new java.io.File(filename).getPath.substring(1)
        })

        absPath match {
          case None => Str(filename)
          case Some(path) => Str(path)
        }
      } else {
        Str(filename)
      }
    }
    case Tuple(Rel, Str(filename)) => Str(new File(".").getCanonicalPath + new File(filename).getPath.substring(1))
    case Tuple(Abs, Str(filename)) => Str(new File(filename).getCanonicalPath)
    case Str(filename) => Str(new File(filename).getCanonicalPath)
    case _ => throw new IllegalArgumentException(s"Not a file descriptor: ${x}. Use:" +
      s"\n\t<name> or (absolute <name>) for absolute path to <name>" +
      s"\n\t(relative <name>) for <name> relative to program path" +
      s"\n\t(relative <uri> <name>) for path relative to module <uri> file path.")
  }
}
