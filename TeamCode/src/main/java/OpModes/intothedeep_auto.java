package OpModes;

import static com.google.blocks.ftcrobotcontroller.hardware.HardwareType.IMU;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.lang.reflect.Array;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous
public class intothedeep_auto extends intothedeep_opmode{

    public Follower follower;
    public Timer pathTimer;
    public Timer opmodeTimer;

    public int pathState = 0;

    public void autonomousPathUpdate(){

    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        super.init();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        pathTimer.resetTimer();
        limelight.start();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ms.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        springToggle.setPosition(springToggleOffPos);
    }

    @Override
        public void start(){
            super.start();
            opmodeTimer.resetTimer();
            setPathState(0);
    }

    @Override
        public void childLoop(){
            super.childLoop();

            if(exAddPower){
                exConstantPID = -1.0;
            }else{
                exConstantPID = 0.0;
            }

            if(veAddPowerDown){
                veConstantPID = -0.8;
            }else if(veHangPower){
                veConstantPID = -2.0;
            }
            else if(veAddPowerUp){
                veConstantPID = 1.0;
            }else{
                veConstantPID = 0.0;
            }

            follower.update();
            autonomousPathUpdate();

            extendo.setPower(slidesPidExtendo.calculatePowerExtendo(slidePositionTargetEx) + exConstantPID);
            slidesPidExtendo.updateEx(extendo.getCurrentPosition(), timeGap);

            verticalPower = 2.0*slidesPidVertical.calculatePowerVertical(slidePositionTargetVe) + veConstantPID;
            ls.setPower(verticalPower);
            ms.setPower(verticalPower);
            rs.setPower(verticalPower);
            slidesPidVertical.updateVe((ms.getCurrentPosition()), timeGap);

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("path state", pathState);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("limelight", limelight.getStatus());
            telemetry.update();
    }
    @Override
    public void stop(){

    }
}
