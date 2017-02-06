package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

/**
 * Created by Simon on 1/6/16.
 */
// RED TEAM


@Autonomous(name = "Shoot")
public class Shoot extends FeRMiLinear {

    @Override
    public void runOpMode() {
        super.initialize(Color.RED);

        setEnc(1500);
        setDrive(0.4);
        sleep(2000);

        flyL.setPower(1);
        flyR.setPower(1);
        sleep(200);
        index.setPosition(0.5);
        sleep(200);
        belt.setPower(1.0);
        sleep(3000);

        stop();
    }

}