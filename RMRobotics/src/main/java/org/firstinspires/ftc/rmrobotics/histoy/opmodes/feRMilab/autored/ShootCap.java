package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;

import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.TIME;

/**
 * Created by Simon on 2/12/17.
 */

@Autonomous(name = "BOTH: Shoot Cap 2")
@Disabled
public class ShootCap extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // sleep 15 seconds
        sleep(20000);

        // drive for .7 seconds on .5 voltage
        drive(TIME, 1300, 0.5);
        setDrive(0);

        // shoot at .95 voltage
        flyL.setPower(0.88);
        flyR.setPower(0.88);
        sleep(700);
        // open indexer
        index.setPosition(0.5);
        sleep(200);
        // turn on belt at voltage 1
        belt.setPower(0.85);
        sleep(2000);
        //stop flywheels and belt
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);

        // END SHOOTING

        //drive for 1.35 seconds on .5 voltage
        drive(TIME, 1000, 0.5);
        setDrive(0);

        //program continues to run until stop button pressed
//        while (opModeIsActive()) {
//            telemetry.addData("yee", "eet");
//            telemetry.update();
//        }

        stop();
    }
}
