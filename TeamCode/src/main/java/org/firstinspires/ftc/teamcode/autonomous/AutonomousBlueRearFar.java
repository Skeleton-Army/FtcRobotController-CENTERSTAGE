package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous - Blue Rear Far", group = "SA_FTC")
public class AutonomousBlueRearFar extends MainAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected InitialPosition initialPosition() {
        return InitialPosition.REAR;
    }

    @Override
    protected Parking parking() { return Parking.FAR; }
}
