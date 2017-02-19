package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by Simon on 2/12/17.
 */

@Autonomous(name = "BOTH: Shoot2")
public class Shoot2 extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // drive for .5 seconds at .4 power
        driveTime(0.4,500);
        setDrive(0);

        // turn on flywheels for .2 seconds at .975 seconds
        flyL.setPower(0.975);
        flyR.setPower(0.975);
        sleep(200);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        // turn on belt
        belt.setPower(1.0);
        sleep(1000);

        stop();
    }
}
