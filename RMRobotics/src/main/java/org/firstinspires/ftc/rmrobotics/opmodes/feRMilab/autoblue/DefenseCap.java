package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autoblue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;

import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.TIME;

/**
 * Created by Simon on 3/18/17.
 */

@Autonomous(name = "BLUE: Defense Cap")
public class DefenseCap extends FeRMiLinear{
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.BLUE, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // sleep for 9 seconds
        // sleep(9000);

        // drive for .1 seconds at .4 voltage
        drive(TIME, 700, 0.5);
        sleep(200);


        // turn on flywheels to .975 voltage
        flyL.setPower(0.83);
        flyR.setPower(0.83);
        sleep(1000);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        //turn on belt
        belt.setPower(0.88);
        sleep(2000);
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);

        // END SHOOTING

        //turn at an angle
        turn(Direction.LEFT, 15, 0.4);

        //drive forward
        drive(TIME, 800, 0.4);

        //face ramp
        turn(Direction.CENTER, 110, 0.4);

        //drive onto ramp
        drive(TIME, 2750, 0.2);

        stop();
    }
}
