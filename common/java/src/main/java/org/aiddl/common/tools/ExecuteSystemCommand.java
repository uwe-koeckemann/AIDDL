package org.aiddl.common.tools;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Simple class to make system calls on a server. 
 * @author Uwe Koeckemann
 *
 */
public class ExecuteSystemCommand {
	
	/**
	 * Execute command <code>cmd</code> in directory <code>dir</code> 
	 * @param dir Directory in which command will be executed.
	 * @param cmd Command to be executed.
	 * @return Array of type String where first element is STDOUT and second element STERR
	 */
	public static String[] callExternal( String dir, String cmd ) {
		try {
			Socket clientSocket;
			clientSocket = new Socket("localhost", 6789);
			DataOutputStream outToServer = new DataOutputStream(clientSocket.getOutputStream());
			BufferedReader inFromServer = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
	
			outToServer.writeBytes(dir + "<CMD>" + cmd + '\n');

			String answer = "";			
			answer = inFromServer.readLine().replace("<LINEBREAK>", "\n");

			clientSocket.close(); 
			String[] ret = answer.split("<ERR>");
			ret[1] = ret[1].replace("<EOF>", "");

			return ret;
		} catch (UnknownHostException e) {
			e.printStackTrace();
			System.exit(1);
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(1);
		}
		return null;
	}
	
	/**
	 * Execute command <code>cmd</code> in directory <code>dir</code> 
	 * @param dir Directory in which command will be executed.
	 * @param cmd Command to be executed.
	 * @return Array of type String where first element is STDOUT and second element STERR
	 */
	public static String[] call( String dir, String cmd ) {

		String s = null;
		
		try {
			ProcessBuilder builder = new ProcessBuilder( cmd.split(" ") );
			builder.directory( new File( dir ).getAbsoluteFile() );
			builder.redirectErrorStream(true);
			
			Process process =  builder.start();

			BufferedReader stdoutReader = new BufferedReader(new
                 InputStreamReader(process.getInputStream()));
 
            BufferedReader stderrReader = new BufferedReader(new
                 InputStreamReader(process.getErrorStream()));
 
            StringBuilder stdout = new StringBuilder();
            StringBuilder stderr = new StringBuilder();
	
            while ((s = stdoutReader.readLine()) != null) {
	        	stdout.append(s);
	        	stdout.append("\n");
	        }
	        while ((s = stderrReader.readLine()) != null) {
	        	stderr.append(s);
	        	stderr.append("\n");
	        }
	             
			process.waitFor();
			
			String[] ret = new String[2];
			ret[0] = stdout.toString();
			ret[1] = stderr.toString();
			return ret;
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return null;
	}
	
	/**
	 * Execute command <code>cmd</code> 
	 * @param cmd Command to be executed.
	 * @return Array of type String where first element is STDOUT and second element STERR
	 */
	public static boolean testIfCommandExists( String cmd ) {	
		try {
			ProcessBuilder builder = new ProcessBuilder( cmd.split(" ") );
//			builder.redirectErrorStream(true);
			Process process =  builder.start();   
			process.waitFor();
			return true;
		} catch (IOException e) {
		} catch (InterruptedException e) {
		}
		return false;
	}
}
