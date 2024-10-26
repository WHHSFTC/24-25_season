package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;

//@Disabled
@Config
@TeleOp
public class intothedeep_tele extends OpMode{

    FtcDashboard dashboard;
    static TelemetryPacket packet;

    DcMotor rf;
    DcMotor lf;
    DcMotor rb;
    DcMotor lb;
    DcMotor extendo;
    Servo alpha;
    Servo beta;
    Servo wrist;
   /* DcMotor rs;
    DcMotor ls;*/

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime totalRunTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double timePerLoop;
    double timeGap;
    double loopCumulativeTime = 0.0;
    double loopCounter = 0.0;
    public static double slideTargetGain = 50.0;
    public static double slideMin = 0.0;
    public static double slideMax = 600.0;
    double slidePositionTarget;

    Gamepad gamepad1prev = new Gamepad();
    Gamepad gamepad2prev = new Gamepad();

    SlidesPID slidesPidExtendo;
    /*SlidesPID slidesPidRight;
    SlidesPID slidesPidLeft;*/


    @Override
    public void init(){

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        slidesPidExtendo = new SlidesPID();
        /*slidesPidLeft = new SlidesPID();
        slidesPidRight = new SlidesPID();*/

        rf = hardwareMap.get(DcMotor.class, "motorRF");
        lf = hardwareMap.get(DcMotor.class, "motorLF");
        rb = hardwareMap.get(DcMotor.class, "motorRB");
        lb = hardwareMap.get(DcMotor.class, "motorLB");
        /*rs = hardwareMap.get(DcMotor.class, "rs");
        ls = hardwareMap.get(DcMotor.class, "ls");*/
        extendo = hardwareMap.get(DcMotor.class, "extendo");


        alpha = hardwareMap.get(Servo.class, "alpha");
        beta = hardwareMap.get(Servo.class, "beta");
        wrist = hardwareMap.get(Servo.class, "wrist");

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //@Override
    public void start(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       /* rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }

    @Override
    final public void loop(){

        slidesPidExtendo.update(extendo.getCurrentPosition(), timeGap);

        timeGap = timer.milliseconds();
        timer.reset();

        timePerLoop = timer.milliseconds();
        timer.reset();
        loopCounter++;
        loopCumulativeTime += timePerLoop;

        if (loopCumulativeTime >= 1000) {
            telemetry.addData("Time per Loop", "Time per Loop: " + loopCumulativeTime/loopCounter);
            loopCounter = 0.0;
            loopCumulativeTime = 0.0;
        }

        //extendo
        if (Math.abs(gamepad2.left_stick_y) > 0.01) {
            slidePositionTarget -= slideTargetGain * gamepad2.left_stick_y;
        }

        if (slidePositionTarget < slideMin) {
            slidePositionTarget = slideMin;
        }
        if (slidePositionTarget > slideMax) {
            slidePositionTarget = slideMax;
        }

        extendo.setPower(slidesPidExtendo.calculatePower(slidePositionTarget));

        //intake
        if(Math.abs(gamepad2.right_stick_y) > 0.01 || Math.abs(gamepad2.right_stick_x) > 0.01){
            alpha.setPosition(((gamepad2.right_stick_y + 1)/2) + ((gamepad2.right_stick_x + 1)/2));
            beta.setPosition(((gamepad2.right_stick_y + 1)/2) - ((gamepad2.right_stick_x + 1)/2));
        }


        //drivetrain
        double y = -gamepad1.left_stick_x; //verticals
        double x = -gamepad1.left_stick_y; //horizontal
        double r = -gamepad1.right_stick_x; //pivot and rotation
        double scalar = 1.0;

        if(gamepad1.left_bumper){
            scalar = 0.6;
        }

        double preRF = r*scalar + y*scalar + x*scalar;
        double preLF = r*scalar + y*scalar - x*scalar;
        double preRB = r*scalar -y*scalar + x*scalar;
        double preLB = r*scalar -y*scalar -x*scalar;

        double max = Math.max(Math.max(Math.max(Math.max(preRF, preRB), preLB), preLF), 1);

        rf.setPower(preRF/max);
        lf.setPower(preLF/max);
        rb.setPower(preRB/max);
        lb.setPower(preLB/max);


        /*telemetry.addData("rs position", rs.getCurrentPosition());
        telemetry.addData("ls position", ls.getCurrentPosition());*/
        telemetry.addData("extendo position", "extendo position: " + extendo.getCurrentPosition());
        telemetry.addData("extendo power", "extendo power: " + extendo.getPower());
        telemetry.addData("time per loop", timePerLoop);
        telemetry.addData("gampead 2 left analog value", "gamepad 2 analog value: " + gamepad2.left_stick_y);
        telemetry.addData("Error Extendo", "Error Extendo: " + (slidePositionTarget - extendo.getCurrentPosition()));
        telemetry.update();

        packet.put("slides target", slidePositionTarget);
        packet.put("slides position extendo", extendo.getCurrentPosition());
        packet.put("slides power extendo", extendo.getPower());
        packet.put("extendo kp", "extendo kp" + SlidesPID.Kp);
        packet.put("extendo kd", "extendo kd" + SlidesPID.Kd);
        packet.put("extendo ki", "extendo ki" + SlidesPID.Ki);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop(){
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        extendo.setPower(0);
    }
}
