package org.firstinspires.ftc.rmrobotics.util.autonav;

import android.util.JsonReader;
import android.util.JsonWriter;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;

public class AutoNavConfig {
    public boolean isRed = false;
    public String firstBeacon = "";
    public String secondBeacon = "";
    public boolean isRight = true;

    public void ReadConfig(FtcRobotControllerActivity act) {
        try {
            InputStream in = act.openFileInput("DarudeAutoNavCfg");
            JsonReader reader = new JsonReader(new InputStreamReader(in, "UTF-8"));
            reader.beginObject();
            reader.nextName();
            isRed = reader.nextBoolean();
            reader.nextName();
            firstBeacon = reader.nextString();
            reader.nextName();
            secondBeacon = reader.nextString();
            reader.nextName();
            isRight = reader.nextBoolean();
            reader.endObject();
            reader.close();
        } catch (FileNotFoundException ex) {
            RobotLog.d("Cannot create config file, creating default");
            // Create default file
            firstBeacon = "Wheels";
            secondBeacon = "Tools";
            WriteConfig(act);
        } catch (IOException ex) {
            RobotLog.d("Cannot create config file");
        }
    }

    public void WriteConfig(FtcRobotControllerActivity act) {
        try {
            OutputStream out = act.openFileOutput("DarudeAutoNavCfg",0);
            JsonWriter writer = new JsonWriter(new OutputStreamWriter(out, "UTF-8"));
            writer.setIndent("  ");
            writer.beginObject();
            writer.name("red").value(isRed);
            writer.name("firstBeacon").value(firstBeacon);
            writer.name("secondBeacon").value(secondBeacon);
            writer.name("isRight").value(isRight);
            writer.endObject();
            writer.close();
        } catch (FileNotFoundException ex) {
            RobotLog.d("Cannot create config file");
        } catch (IOException ex) {
            RobotLog.d("Cannot create config file");
        }
    }
}
