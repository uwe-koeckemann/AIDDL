package org.aiddl.external.scala.prolog.tools

import java.io.{BufferedReader, File, IOException, InputStreamReader}
import scala.jdk.CollectionConverters.*

/**
 * Helper class that execute a system command. Used to run Prolog.
 */
object ExecuteSystemCommand {

  /**
   * Call a command cmd in a directory dir
   * @param dir directory to run the command in
   * @param cmd the command to run
   * @return optional pair of standard out and standard error
   */
  def call( dir: String, cmd: String ): Option[(String, String)] = {
    var s: String = ""

    try {
      val args = cmd.split(" ").toList.asJava
      val builder = new ProcessBuilder( args )
      builder.directory( new File( dir ).getAbsoluteFile() )
      builder.redirectErrorStream(true)

      val process = builder.start();

      val stdoutReader = new BufferedReader(new
          InputStreamReader(process.getInputStream()))

      val stderrReader = new BufferedReader(new
          InputStreamReader(process.getErrorStream()))

      val stdout = new StringBuilder()
      val stderr = new StringBuilder()

      while { s = stdoutReader.readLine(); s != null } do {
        stdout.append(s)
        stdout.append("\n")
      }
      while { s = stderrReader.readLine(); s != null } do {
        stderr.append(s)
        stderr.append("\n")
      }

      process.waitFor();

      val sout = stdout.toString()
      val serr = stderr.toString()
      if ( sout == "" && serr == "" ) None
      else Some((stdout.toString(), stderr.toString()))
    } catch {
      case e: IOException => e.printStackTrace(); None
      case e: InterruptedException => e.printStackTrace(); None
    }
  }
}
