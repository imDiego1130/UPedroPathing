package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001968686354;
        ThreeWheelConstants.strafeTicksToInches = .001968686354;
        ThreeWheelConstants.turnTicksToInches = .0019520996384313307;
        ThreeWheelConstants.leftY = 4.1617984236;
        ThreeWheelConstants.rightY = -4.1617984236;
        ThreeWheelConstants.strafeX = -0.3;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "lF";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rB";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rF";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




