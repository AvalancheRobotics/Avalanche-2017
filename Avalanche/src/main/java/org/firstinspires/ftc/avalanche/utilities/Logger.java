package org.firstinspires.ftc.avalanche.utilities;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;

/**
 * Created by Keith on 10/23/2016.
 */

public class Logger {

    File log;

    public Logger(String fileName)
    {
        log = new File("sdcard/" + fileName + ".log");
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
            }
        }
    }

    public void log(String text)
    {
        try
        {
            //BufferedWriter for performance, true to set append to file flag
            PrintStream ps = new PrintStream(log);
            ps.println(text);
            ps.close();
        }
        catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void log(Exception ex)
    {
        try
        {
            PrintStream ps = new PrintStream(log);
            ex.printStackTrace(ps);
            ps.close();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }

    }
}
