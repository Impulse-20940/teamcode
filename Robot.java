package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot{
    //DcMotor lift = null;
    //DcMotor manipulator = null;
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode L) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
    }
    public void get_members() {
        //lift = hardwareMap.get(DcMotor.class, "reechniy_lift");
        //manipulator = hardwareMap.get(DcMotor.class, "manipulator");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //manipulator.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void go_bytime(double axial, double lateral, double yaw, double time) {
        get_members();
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        runtime.reset();
        while (L.opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void go_byenc_right(double ticks) {
        get_members();
        while ((Math.abs(leftFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            //double enc = (Math.abs(rightFrontDrive.getCurrentPosition())+Math.abs(rightBackDrive.getCurrentPosition()))/2;
            //double er = ticks-enc;
            //double kp = 0;//here is coeff
            //double p_reg = er*kp;
             double axial = 0;
            double lateral = 0.5;//write lateral = er*kp
            double yaw = 0;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(leftFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(leftFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(leftFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(leftFrontDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = 1;
                }
            }
            telemetry.addData("Now is", "%7d :%7d",
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
    public void teleop() {
        get_members();
        double max;
        //axiall это axial для реечного лифта
        //axialm это axial для манипулятора(качельки) если это так роботает конечно))))
        //double axiall = gamepad2.left_stick_y*0.1;
        //double axialm = gamepad2.right_stick_y*0.2;

        double axial   = -gamepad1.left_stick_y*0.5;
        double lateral =  gamepad1.left_stick_x*0.5;
        double yaw     =  -gamepad1.right_stick_x*0.5;

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
}
