

    package org.firstinspires.ftc.rmrobotics.opmodes.SandstoRM;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.I2cAddr;
    import com.qualcomm.robotcore.hardware.I2cDevice;
    import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
    import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
    import com.qualcomm.robotcore.hardware.Servo;

    import java.util.ArrayList;
    import java.util.Collections;
    import java.util.List;

    /**
     * Created by ROTOM on 12/8/2016.
     */

    @TeleOp(name="DarudeTele", group="SandstoRM")
    public class DarudeTele extends OpMode {
        private double servoHeight;
        private DcMotor wheelFL;
        private DcMotor wheelFR;
        private DcMotor wheelBL;
        private DcMotor wheelBR;
        private DcMotor shootL;
        private DcMotor shootR;
        private DcMotor harvest;
        private DcMotor lift;
        private Servo beaconL;
        private Servo index;
        private Servo grabberL;
        private Servo grabberR;

        public void init() {

            wheelFL = hardwareMap.dcMotor.get("wheelFL");
            wheelFR = hardwareMap.dcMotor.get("wheelFR");
            wheelBL = hardwareMap.dcMotor.get("wheelBL");
            wheelBR = hardwareMap.dcMotor.get("wheelBR");
            shootL = hardwareMap.dcMotor.get("shootL");
            shootR = hardwareMap.dcMotor.get("shootR");
            harvest = hardwareMap.dcMotor.get("harvest");
            lift = hardwareMap.dcMotor.get("lift");
            wheelFR.setDirection(DcMotorSimple.Direction.REVERSE);
            wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);

            beaconL = hardwareMap.servo.get("beaconL");
            index = hardwareMap.servo.get("index");
            servoHeight = .5;
            beaconL.setPosition(servoHeight);
            index.setPosition(.5);
//            grabberL.setPosition(0);
//            grabberR.setPosition(0);
//            grabberR.setDirection(Servo.Direction.REVERSE);
        }

        public void loop() {

            double forward = gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;
            double max = 1;
            List l = new ArrayList<>();
            l.add(Math.abs(forward + strafe + rotate));
            l.add(Math.abs(forward - strafe - rotate));
            l.add(Math.abs(forward - strafe + rotate));
            l.add(Math.abs(forward + strafe - rotate));
            if ((double) Collections.max(l) > 1) {
                max = (double) Collections.max(l);
            }
            wheelFL.setPower((-forward - strafe + rotate) / max);
            wheelFR.setPower((-forward + strafe - rotate) / max);
            wheelBL.setPower((-forward + strafe + rotate) / max);
            wheelBR.setPower((-forward - strafe - rotate) / max);

            if (gamepad2.right_bumper) {
                shootL.setPower(1.0);
                shootR.setPower(1.0);
            } else if (gamepad2.left_bumper) {
                shootL.setPower(-1.0);
                shootR.setPower(-1.0);
            } else {
                shootL.setPower(0.0);
                shootR.setPower(0.0);
            }

            if (gamepad1.right_bumper) {
                harvest.setPower(-1.0);
            } else if (gamepad1.left_bumper) {
                harvest.setPower(1.0);
            } else {
                harvest.setPower(0.0);
            }

            if ((gamepad1.left_trigger)!= 0 && gamepad1.right_trigger == 0) {
                lift.setPower(1.0);
            } else if ((gamepad1.right_trigger) != 0 && gamepad1.left_trigger == 0) {
                lift.setPower(-1.0);
            } else {
                lift.setPower(0.0);
            }
            if (gamepad1.dpad_down)
            {
                if (servoHeight <= 1)
                {
                    servoHeight += .003;
                }
            }
            if (gamepad1.dpad_up)
            {
                if (servoHeight >= .35)
                {
                    servoHeight -= .003;
                }
            }
            beaconL.setPosition(servoHeight);

            if (gamepad2.y) {
                    index.setPosition(1.1);
            }else{index.setPosition(.5);}


//            if (gamepad1.x) {
//                grabberL.setPosition(0);
//                grabberR.setPosition(0);
//            }
//            if (gamepad1.a) {
//                grabberL.setPosition(0.55);
//                grabberR.setPosition(0.55);
//            }
//            if (gamepad1.b) {
//                grabberL.setPosition(1);
//                grabberR.setPosition(1);
//            }

        }
    }


