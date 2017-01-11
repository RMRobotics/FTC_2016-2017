package org.firstinspires.ftc.rmrobotics.opmodes;

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
public class colorline extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorAcache;
    byte[] colorBcache;

    I2cDevice colorA;
    I2cDevice colorB;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorA = hardwareMap.i2cDevice.get("colorLeft");
        colorB = hardwareMap.i2cDevice.get("colorRight");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x72), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, I2cAddr.create8bit(0x70), false);

        colorAreader.engage();
        colorBreader.engage();

        colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        colorBreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);

        //display values
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData("2 #B", colorBcache[0] & 0xFF);

        telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());
        telemetry.addData("4 B", colorBreader.getI2cAddress().get8Bit());
    }
}
