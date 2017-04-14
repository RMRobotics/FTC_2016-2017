package org.firstinspires.ftc.rmrobotics.histoy.external;

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

@Autonomous(name = "colorValues")
@Disabled
public class colorline extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorAcache;
    byte[] colorBcache;
    byte[] colorCcache;
    byte[] colorDcache;

    I2cDevice colorA;
    I2cDevice colorB;
    I2cDevice colorC;
    I2cDevice colorD;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;
    I2cDeviceSynch colorCreader;
    I2cDeviceSynch colorDreader;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorA = hardwareMap.i2cDevice.get("colorLeft");
        colorB = hardwareMap.i2cDevice.get("colorRight");
        colorC = hardwareMap.i2cDevice.get("colorBack");
        colorD = hardwareMap.i2cDevice.get("colorCenter");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x72), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, I2cAddr.create8bit(0x70), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x50), false);
        colorDreader = new I2cDeviceSynchImpl(colorD, I2cAddr.create8bit(0x52), false);

        colorAreader.engage();
        colorBreader.engage();
        colorCreader.engage();
        colorDreader.engage();

        colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        colorBreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        colorCreader.write8(3, 0);
        colorDreader.write8(3, 0);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);
        colorDcache = colorDreader.read(0x04, 1);

        //display values
        //telemetry.addData("LEFT", colorAcache[0] & 0xFF);
        //telemetry.addData("RIGHT", colorBcache[0] & 0xFF);
        telemetry.addData("BACK 0x04", colorCreader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("BACK 0x05", colorCreader.read(0x05, 1)[0] & 0xFF);
        telemetry.addData("BACK 0x06", colorCreader.read(0x06, 1)[0] & 0xFF);
        telemetry.addData("BACK 0x07", colorCreader.read(0x07, 1)[0] & 0xFF);
        telemetry.addData("BACK 0x08", colorCreader.read(0x08, 1)[0] & 0xFF);
        telemetry.addData("CENTER 0x04", colorDreader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("CENTER 0x05", colorDreader.read(0x05, 1)[0] & 0xFF);
        telemetry.addData("CENTER 0x06", colorDreader.read(0x06, 1)[0] & 0xFF);
        telemetry.addData("CENTER 0x07", colorDreader.read(0x07, 1)[0] & 0xFF);
        telemetry.addData("CENTER 0x08", colorDreader.read(0x08, 1)[0] & 0xFF);

//        telemetry.addData("5 A", colorAreader.getI2cAddress().get8Bit());
//        telemetry.addData("6 B", colorBreader.getI2cAddress().get8Bit());
//        telemetry.addData("7 A", colorCreader.getI2cAddress().get8Bit());
//        telemetry.addData("8 B", colorDreader.getI2cAddress().get8Bit());
    }
}
