package frc.robot.Autonomous;

import frc.robot.subsystems.*;
import frc.utils.SwerveUtils;
import frc.robot.*;

import java.io.IOException;
import java.nio.file.Path;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
public class Auto {

    public class newauto extends SequentialCommandGroup{
        public newauto(SwerveUtils s_Swerve){
            

        }
    }

    


    // public Command followPathCommand(PathPlannerTrajectory path) {
    //     return new SequentialCommandGroup(PathPlannerTrajectory );
    // }

    // public static Command test() {
    //     return new SequentialCommandGroup(

    //     );
}    




    

