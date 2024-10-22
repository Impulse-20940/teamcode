//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//@TeleOp(name="MainTeleOp")
//public class BasicOmniOpMode extends LinearOpMode {
//    //Here is all robot devices and used classes
//    Robot R = new Robot();
//    ElapsedTime runtime = new ElapsedTime();
//    DcMotor leftFrontDrive = null;
//    DcMotor leftBackDrive = null;
//    DcMotor rightFrontDrive = null;
//    DcMotor rightBackDrive = null;
//    @Override
//    public void runOpMode() {
//        R.get_members();
//        // Ждем нажатия кнопки старт
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        runtime.reset();
//
//        // Работаем пока не нажат стоп
//        while (opModeIsActive()) {
//            double max;
//
//            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
//            double axial   = -gamepad1.left_stick_y*0.5;  // Note: pushing stick forward gives negative value
//            double lateral =  gamepad1.left_stick_x*0.5;
//            double yaw     =  -gamepad1.right_stick_x*0.5;
//            double axialm = gamepad1.right_stick_y*0.2;
//
//            // Combine the joystick requests for each axis-motion to determine each wheel's power.
//            // Set up a variable for each drive wheel to save the power level for telemetry.
//            double ManipulatorFrontPower = axialm;            double leftFrontPower  = axial + lateral + yaw;
//            double rightFrontPower = axial - lateral - yaw;
//            double leftBackPower   = axial - lateral + yaw;
//            double rightBackPower  = axial + lateral - yaw;
//
//            // Normalize the values so no wheel power exceeds 100%
//            // This ensures that the robot maintains the desired motion.
//            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));
//
//            if (max > 1.0) {
//                leftFrontPower  /= max;
//                rightFrontPower /= max;
//                leftBackPower   /= max;
//                rightBackPower  /= max;
//            }
//            //Шлем значения с переменных на моторы
//            leftFrontDrive.setPower(leftFrontPower);
//            rightFrontDrive.setPower(rightFrontPower);
//            leftBackDrive.setPower(leftBackPower);
//            rightBackDrive.setPower(rightBackPower);
//
//            // Показываем время использования и скорость моторов в приложении
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.update();
//        }
//        }
//    }
