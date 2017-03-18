package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autoblue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

import static org.firstinspires.ftc.rmrobotics.util.Drive.ENCODER;
import static org.firstinspires.ftc.rmrobotics.util.Drive.TIME;

/**
 * Created by Simon on 3/17/17.
 */

@Autonomous(name = "BLUE: Shoot Ramp")
public class ShootRamp extends FeRMiLinear{
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.BLUE, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // sleep for 10 seconds
        // sleep(10000);

        // drive for .1 seconds at .4 power
        drive(TIME, 700, 0.5);
        sleep(200);

        // turn on flywheels to .975 power
        flyL.setPower(0.985);
        flyR.setPower(0.985);
        sleep(200);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        //turn on belt
        belt.setPower(1.0);
        sleep(2000);
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);


        //turn at an angle
        turn(Direction.LEFT, 45, 0.4);

        //drive forward
        drive(TIME, 2000, 0.4);

        //face ramp
        // turn(Direction.CENTER, 130, 0.4);

        //drive onto ramp
        drive(ENCODER, 2500, 0.4);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

}
