package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot{
    //DcMotor lift = null;
    //DcMotor manipulator = null;
    double ticks;
    Servo klesh;
    DcMotor man;
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
        man = hardwareMap.get(DcMotor.class, "m");
        klesh = hardwareMap.get(Servo.class, "kl");
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
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double enc = rightBackDrive.getCurrentPosition();
            double er = ticks-enc;
            double kp = 0.001;//here is coeff
            double p_reg = er*kp;

            double axial = 0;
            double lateral = p_reg;
            double yaw = 0;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
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
    public void go_byenc_left(double ticks) {
        get_members();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double enc = rightBackDrive.getCurrentPosition();
            double er = ticks-enc;
            double kp = 0.001;//here is coeff
            double p_reg = er*kp;

            double axial = 0;
            double lateral = -p_reg;
            double yaw = 0;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
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
    public void go_byenc_forward(double ticks) {
        get_members();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double enc = rightBackDrive.getCurrentPosition();
            double er = ticks-enc;
            double kp = 0.001;//here is coeff
            double p_reg = er*kp;

            double axial = p_reg;
            double lateral = 0;
            double yaw = 0;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
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
    public void go_byenc_backward(double ticks) {
        get_members();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double enc = rightBackDrive.getCurrentPosition();
            double er = ticks-enc;
            double kp = 0.001;//here is coeff
            double p_reg = er*kp;

            double axial = -p_reg;
            double lateral = 0;
            double yaw = 0;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
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
    public void turn_byenc_right(double ticks) {
        get_members();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double enc = rightBackDrive.getCurrentPosition();
            double er = ticks-enc;
            double kp = 0.001;//here is coeff
            double p_reg = er*kp;

            double axial = 0;
            double lateral = 0;
            double yaw = p_reg;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
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
    public void turn_byenc_left(double ticks) {
        get_members();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double enc = rightBackDrive.getCurrentPosition();
            double er = ticks-enc;
            double kp = 0.001;//here is coeff
            double p_reg = er*kp;

            double axial = 0;
            double lateral = 0;
            double yaw = -p_reg;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - Math.abs(rightBackDrive.getCurrentPosition())) > 300) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
                while ((Math.abs(rightBackDrive.getCurrentPosition()) - Math.abs(rightFrontDrive.getCurrentPosition())) > 300) {
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
        double axialm = -gamepad2.right_stick_y*0.4+0.2;
        double axial   = -gamepad1.left_stick_y*0.5;
        double lateral =  gamepad1.left_stick_x*0.5;
        double yaw     =  -gamepad1.right_stick_x*0.5;
        double servo_position = gamepad2.left_stick_y+0.2;
        //double liftPower = axiall;
        double ManPower = axialm;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max = Math.max(max, Math.abs(ManPower));
        //max = Math.max(max, Math.abs(liftPower));

        if (max > 1.0) {
            //liftPower /=max;
            ManPower /= max;
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        //lift.setPower(liftPower);
        klesh.setPosition(servo_position);
        man.setPower(ManPower);
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
