package org.firstinspires.ftc.avalanche.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintStream;

/**
 * Allows logging to a specified file that can easily be accessed in the phones storage directory.
 * Files are saved as <pre>.txt</pre> unless otherwise specified.
 *
 * @author Keith
 */

public class Logger {

    File log;
    LinearOpMode opmode;

    /**
     * Created a Logger with a specified file name.
     * @param fileName  The name of the file to save to, without the extension.
     */
    public Logger(String fileName)
    {
        log = new File("sdcard/" + fileName + ".txt");
        if (!log.exists())
        {
            try
            {
                log.createNewFile();
            }
            catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
                if (opmode != null)
                {
                    opmode.telemetry.addData("Exception", e);
                }
            }
        }
    }

    /**
     * Creates a Logger with a specified fileName and file extension (via a variable).
     * @param fileName  The name of the file to save to, without the extension.
     * @param logExt  If true, sets the file extention as a <pre>.log</pre> file, otherwise uses
     *                a <pre>.txt</pre> file
     */
    public Logger(String fileName, boolean logExt)
    {
        log = new File(logExt ? "sdcard/" + fileName + ".log" : "sdcard/" + fileName + ".txt");
        if (!log.exists())
        {
            try
            {
                log.createNewFile();
            }
            catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
                if (opmode != null)
                {
                    opmode.telemetry.addData("Exception", e);
                }
            }
        }
    }

    /**
     * Sets the opmode used for debugging.
     * @param opmode The opmode to call telemetry on.
     */
    public void setOpmode(LinearOpMode opmode)
    {
        this.opmode = opmode;
    }

    /**
     * Append a string to the log file.
     * @param text The string to append.
     */
    public void log(String text)
    {
        try
        {
            //write text to file
            FileOutputStream fos = new FileOutputStream(log, true);
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(fos);
            outputStreamWriter.write(text);
            //start a new line
            String separator = System.getProperty("line.separator");
            outputStreamWriter.append(separator);
            outputStreamWriter.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
            //If opmode is defined, log the error to telemetry
            if (opmode != null)
            {
                opmode.telemetry.addData("Exception", e);
            }
        }
    }

    /**
     * Append a stacktrace of an exception to the log file.
     * @param ex the exception whose stacktrace to append.
     */
    public void log(Exception ex)
    {
        try
        {
            //write the exception
            PrintStream ps = new PrintStream(new FileOutputStream(log, true), true);
            ex.printStackTrace(ps);
            ps.close();
        }
        catch (Exception e)
        {
            e.printStackTrace();
            //if opmode is defined, log the error to telemetry
            if (opmode != null)
            {
                opmode.telemetry.addData("Exception", e);
            }
        }
    }
}
