package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SmartDash {
    private static boolean isInCompetition = DriverStation.isFMSAttached() && DriverStation.isDSAttached();

    private SmartDash(){throw new UnsupportedOperationException();}
 
/**
 * 
 * @param key The string key to show on SmartDashboard
 * @param number The number value to log to SmartDashboard
 * @param showInComp True or False of whether or not to show this value during competition
 */ 
    public static void putNumber(String key, double number, boolean showInComp){
        if (isInCompetition && showInComp){
            
            SmartDashboard.putNumber(key, number);
        }
        else if (!isInCompetition){
            SmartDashboard.putNumber(key, number);
        }
  
    }

/**
 * 
 * @param key The string key to show on SmartDashboard
 * @param bool The boolean value to log to SmartDashboard
 * @param showInComp True or False of whether or not to show this value during competition
 */


    public static void putBoolean(String key, boolean bool, boolean showInComp){
        if (isInCompetition && showInComp){
            SmartDashboard.putBoolean(key, bool);
        }else if (!isInCompetition){
            SmartDashboard.putBoolean(key, bool);
        }

    }

/**
 * 
 * @param key The string key to show on SmartDashboard
 * @param string The string value to log to SmartDashboard
 * @param showInComp True or False of whether or not to show this value during competition
 */

    public static void putString(String key, String string, boolean showInComp){
        if (isInCompetition && showInComp){
            SmartDashboard.putString(key, string);
        }else if (!isInCompetition){
            SmartDashboard.putString(key, string);
        }
    }
}
