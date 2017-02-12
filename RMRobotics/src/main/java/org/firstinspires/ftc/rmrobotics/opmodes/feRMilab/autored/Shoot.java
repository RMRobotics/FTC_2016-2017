package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

/**
 * Created by Simon on 1/6/16.
 */
// RED TEAM


@Autonomous(name = "RED: Shoot")
public class Shoot extends FeRMiLinear {

    @Override
    public void runOpMode() {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER);

        setDrive(0.4);
        sleep(100);
        setDrive(0);
        flyL.setPower(0.975);
        flyR.setPower(0.975);
        sleep(200);
        index.setPosition(0.5);
        sleep(200);
        belt.setPower(1.0);
        sleep(3000);

        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);
        stop();
    }

}