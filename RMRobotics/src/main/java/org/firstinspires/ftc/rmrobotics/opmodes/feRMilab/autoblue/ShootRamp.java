package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autoblue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;

import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.TIME;

/**
 * Created by Simon on 3/17/17.
 */
// BLUE TEAM

@Autonomous(name = "BLUE: Shoot Ramp")
@Disabled
public class ShootRamp extends FeRMiLinear{
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.BLUE, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // sleep for 15 seconds
        sleep(15000);

        // drive for 1.2 seconds at 0.5 voltage
        drive(TIME, 1200, 0.5);
        sleep(200);

        // turn on flywheels to 0.94 voltage
        flyL.setPower(0.94);
        flyR.setPower(0.94);
        sleep(2000);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        // turn on belt
        belt.setPower(0.88);
        sleep(2000);
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);

        // END SHOOTING

        drive(TIME, 500, 0.4);

        // turn at an angle
        turn(Direction.LEFT, 45, 0.4);

        // drive forward
        drive(TIME, 1000, 0.4);

        // face ramp
        turn(Direction.CENTER, 90, 0.4);

        //drive onto ramp
        drive(TIME, 1700, 0.3);

        while (opModeIsActive()){
            addTelemetry();
        }
        stop();
    }

}
