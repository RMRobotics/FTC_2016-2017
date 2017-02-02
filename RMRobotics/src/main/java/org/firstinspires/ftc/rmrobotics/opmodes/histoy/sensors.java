package org.firstinspires.ftc.rmrobotics.opmodes.histoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Peter on 11/29/16.
 */

@Autonomous(name = "sensors")
@Disabled
public class sensors extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorAcache;
    byte[] colorCcache;
    byte[] colorLinecache;

    I2cDevice colorA;
    I2cDevice colorC;
    I2cDevice colorLine;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;
    I2cDeviceSynch colorLinereader;

    byte[] rangeAcache;
    byte[] rangeCcache;

    I2cDevice rangeA;
    I2cDevice rangeC;
    I2cDeviceSynch rangeAreader;
    I2cDeviceSynch rangeCreader;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorA = hardwareMap.i2cDevice.get("cL");
        colorC = hardwareMap.i2cDevice.get("cR");
        colorLine = hardwareMap.i2cDevice.get("colorLine");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x50), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x52), false);
        colorLinereader = new I2cDeviceSynchImpl(colorLine, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();
        colorLinereader.engage();

        colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        colorLinereader.write8(3,0);

        rangeA = hardwareMap.i2cDevice.get("rL");
        rangeC = hardwareMap.i2cDevice.get("rR");

        rangeAreader = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x60), false);
        rangeCreader = new I2cDeviceSynchImpl(rangeC, I2cAddr.create8bit(0x62), false);

        rangeAreader.engage();
        rangeCreader.engage();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorAcache = colorAreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);
        colorLinecache = colorLinereader.read(0x04, 1);

        //display values
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);
        telemetry.addData("3 #L", colorLinecache[0] & 0xFF);

        telemetry.addData("4 A", colorAreader.getI2cAddress().get8Bit());
        telemetry.addData("5 A", colorCreader.getI2cAddress().get8Bit());
        telemetry.addData("6 A", colorLinereader.getI2cAddress().get8Bit());

        rangeAcache = rangeAreader.read(0x04, 2);  //Read 2 bytes starting at 0x04
        rangeCcache = rangeCreader.read(0x04, 2);

        int RUS = rangeCcache[0] & 0xFF;   //Ultrasonic value is at index 0. & 0xFF creates a value between 0 and 255 instead of -127 to 128
        int LUS = rangeAcache[0] & 0xFF;
        int RODS = rangeCcache[1] & 0xFF;
        int LODS = rangeAcache[1] & 0xFF;

        //display values
        telemetry.addData("1 A US", LUS);
        telemetry.addData("2 A ODS", LODS);
        telemetry.addData("3 C US", RUS);
        telemetry.addData("4 C ODS", RODS);
    }
}
