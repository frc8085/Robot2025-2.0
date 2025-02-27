package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightReefSubsystem;

/**
 * AutoAlignToAprilTagCommand uses vision data from the LimelightReefSubsystem
 * to align the robot with an AprilTag target by applying both rotation and
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  
 *  
 *  

     

     * 
     * 
     * 
     * 
     * 
     * 
     * 
     *  * 
       
     * 
     * 
     *      
        //

            
        //

            
        //

            
        //

            
        //

            
        //

            
        // 
        /

        // 
        /

        // 
        //   

        //  
        // 
        //  

        // 
        // 
        // 
        driveSubsystem.drive(translationSpeed, 1.0, 0.0, rota orrection, false)
        // 
        // 
        // 
        // 
    } 
        // 
        // 
        // 

        // 
        // 
        // 
    @Override
        // 
    public void end(boolean interrupted) {
        // 
        // Stop the robot when the command ends or is interrupted.
        // 
        driveSubsystem.drive(0, 0, 0, 0, false);
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
        // 
    }

 




        
        // 
        // 
        // 