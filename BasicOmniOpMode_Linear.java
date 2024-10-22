package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp_newDev")

public class BasicOmniOpMode_Linear extends LinearOpMode {
    //Манипулятор это качелька
    //private DcMotor lift = null;
    //private DcMotor manipulator = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    public void runOpMode() {
        //lift = hardwareMap.get(DcMotor.class, "reechniy_lift");
        //manipulator = hardwareMap.get(DcMotor.class, "manipulator");
        Robot R = new Robot();
        R.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);
        R.get_members();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double max;
            //axiall это axial для реечного лифта
            //axialm это axial для манипулятора(качельки) если это так роботает конечно))))
            //double axiall = gamepad2.left_stick_y*0.2;
            //double axialm = gamepad2.right_stick_y*0.2;

            double axial   = -gamepad1.left_stick_y*0.5;
            double lateral =  gamepad1.left_stick_x*0.5;
            double yaw     =  gamepad1.right_stick_x*0.5;

            //double liftPower = axiall;
            //double ManipulatorPower = axialm;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //max = Math.max(max, Math.abs(ManipulatorPower));
            //max = Math.max(max, Math.abs(liftPower));

            if (max > 1.0) {
                //liftPower /=max;
                //ManipulatorPower /= max;
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            //lift.setPower(liftPower);
            //manipulator.setPower(ManipulatorPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("manipulator", "%4.2f" , ManipulatorPower);
            //telemetry.addData("reechniy_lift", "$4.2f", liftPower);
            telemetry.update();
        }
    }}
