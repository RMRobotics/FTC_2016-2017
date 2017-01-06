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

@Autonomous(name = "color values")
public class colorline extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] colorAcache;

    I2cDevice colorA;
    I2cDeviceSynch colorAreader;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorA = hardwareMap.i2cDevice.get("cL");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x50), false);

        colorAreader.engage();

        colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        colorAcache = colorAreader.read(0x04, 1);

        //display values
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);

        telemetry.addData("4 A", colorAreader.getI2cAddress().get8Bit());
    }
}
