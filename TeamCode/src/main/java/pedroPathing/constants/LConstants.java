package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .0005;
        ThreeWheelIMUConstants.strafeTicksToInches = .0005;
        ThreeWheelIMUConstants.turnTicksToInches = .001989436789;
        ThreeWheelIMUConstants.leftY = 5.5;
        ThreeWheelIMUConstants.rightY = -5.5;
        ThreeWheelIMUConstants.strafeX = -5.0;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "motorRF";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "motorRB";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "motorLB";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    }
}