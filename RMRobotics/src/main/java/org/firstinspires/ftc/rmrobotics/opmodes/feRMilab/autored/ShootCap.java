package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by Simon on 2/12/17.
 */

@Autonomous(name = "BOTH: Shoot Cap 2")
public class ShootCap extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // sleep 10 seconds
        sleep(10000);

        // drive for .7 seconds on .5 power
        setDrive(0.5);
        sleep(700);
        setDrive(0);

        // shoot at .95 power
        flyL.setPower(0.95);
        flyR.setPower(0.95);
        // wait for .2 seconds for flywheel to speed up
        sleep(200);
        // open indexer
        index.setPosition(0.5);
        sleep(200);
        // turn on belt at power 1
        belt.setPower(1.0);
        sleep(3000);
        //stop flywheels and belt
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);

        //drive for 1.35 seconds on .5 power
        setDrive(.5);
        sleep(1350);

        //stop drive
        setDrive(0);

        //program continues to run until stop button pressed
        while (opModeIsActive()) {
            telemetry.addData("yee", "eet");
            telemetry.update();
        }

        stop();
    }
}
