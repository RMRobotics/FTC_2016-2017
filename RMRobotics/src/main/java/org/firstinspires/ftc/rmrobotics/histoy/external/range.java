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

@Autonomous(name = "range values")
@Disabled
public class range extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] rangeAcache;

    I2cDevice rangeA;
    I2cDeviceSynch rangeAreader;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        rangeA = hardwareMap.i2cDevice.get("range");
        rangeAreader = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x60), false);
        rangeAreader.engage();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        rangeAcache = rangeAreader.read(0x04, 2);  //Read 2 bytes starting at 0x04

        int LUS = rangeAcache[0] & 0xFF;
        int LODS = rangeAcache[1] & 0xFF;

        //display values
        telemetry.addData("1 A US", LUS);
        telemetry.addData("2 A ODS", LODS);
    }
}
