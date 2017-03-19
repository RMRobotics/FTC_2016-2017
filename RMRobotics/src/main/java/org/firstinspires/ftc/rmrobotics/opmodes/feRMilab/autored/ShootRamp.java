package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

import static org.firstinspires.ftc.rmrobotics.util.Drive.TIME;

/**
 * Created by Simon on 2/12/17.
 */

@Autonomous(name = "RED: Shoot Ramp")
public class ShootRamp extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        voltage = flyMC.getVoltage()*-0.1242 + 2.421 + 0.1;
        telemetry.addData("voltage", voltage);
        telemetry.update();

        // sleep for 10 seconds
         sleep(10000);

        // drive for .1 seconds at .4 voltage
        drive(TIME, 1300, 0.5);
        sleep(200);

        // turn on flywheels to .985 voltage
//        voltage = flyMC.getVoltage()*-0.1242 + 2.421 + 0.05;
        flyL.setPower(voltage);
        flyR.setPower(voltage);
        sleep(1000);

        // open indexer
        index.setPosition(0.5);
        sleep(200);

        //turn on belt
        belt.setPower(0.85);
        sleep(2000);
        flyL.setPower(0);
        flyR.setPower(0);
        belt.setPower(0);

        // END SHOOTING

        //turn at an angle
        turn(Direction.RIGHT, -45, 0.4);

        //drive forward
        drive(TIME, 1000, 0.4);

        //face ramp
        turn(Direction.CENTER, -90, 0.4);

        //drive onto ramp
        drive(TIME, 1750, 0.4);

//        while (opModeIsActive()){
//            addTelemetry();
//        }
    }

}



