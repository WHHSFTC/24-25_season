package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class SimpleAuto extends LinearOpMode {
    DcMotor rf;
   DcMotor lb;
   DcMotor rb;
   DcMotor lf;
   DcMotor extendo;
   DcMotor ls;
   DcMotor rs;
   DcMotor ms;
   @Override
   public void runOpMode(){
       rf = hardwareMap.get(DcMotor.class, "motorRF");
       lf = hardwareMap.get(DcMotor.class, "motorLF");
       rb = hardwareMap.get(DcMotor.class, "motorRB");
       lb = hardwareMap.get(DcMotor.class, "motorLB");

       lf.setDirection(DcMotor.Direction.REVERSE);
       lb.setDirection(DcMotor.Direction.REVERSE);
       rf.setDirection(DcMotor.Direction.FORWARD);
       rb.setDirection(DcMotor.Direction.FORWARD);

       rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       waitForStart();
       moveRobot(0,1,0);
       sleep(500);
   }

    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        lf.setPower(leftFrontPower);
        rf.setPower(rightFrontPower);
        lb.setPower(leftBackPower);
        rb.setPower(rightBackPower);
    }


}
