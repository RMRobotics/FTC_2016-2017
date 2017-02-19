package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by Simon on 2/12/17.
 */
@Autonomous(name = "RED: Shoot Ramp")
public class ShootRamp extends FeRMiLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        // drive for .1 seconds at .4 power
        driveTime(0.4, 100);
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

        //turn at an angle
        turnCorner(-45, 0.4, Direction.RIGHT);

        //drive forward
        driveEncoder(700, 0.4);

        //face ramp
        turnCenter(-130, 0.4);

        //drive onto ramp
        driveEncoder(2500, 0.4);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

}



