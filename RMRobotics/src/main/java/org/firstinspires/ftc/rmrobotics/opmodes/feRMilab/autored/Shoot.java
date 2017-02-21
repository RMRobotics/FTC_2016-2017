package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

import static org.firstinspires.ftc.rmrobotics.util.Drive.TIME;

/**
 * Created by Simon on 1/6/16.
 */
// RED TEAM


@Autonomous(name = "RED: Shoot")
public class Shoot extends FeRMiLinear {

    @Override
    public void runOpMode() {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // drive for .1 seconds at .4 power
        drive(TIME, 100, 0.4);
        sleep(200);

        // turn on flywheels to .975 power
        flyL.setPower(0.975);
        flyR.setPower(0.975);
        sleep(200);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        //turn on belt
        belt.setPower(1.0);
        sleep(3000);

        stop();
    }

}