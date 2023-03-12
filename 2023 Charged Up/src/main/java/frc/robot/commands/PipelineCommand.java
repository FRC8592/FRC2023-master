package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Vision;

public class PipelineCommand extends Command {
    private Pipeline pipeline;

    public enum Pipeline {
        CONE(Constants.CONE_PIPELINE),
        CUBE(Constants.CUBE_PIPELINE),
        APRIL_TAG(Constants.APRILTAG_PIPELINE),
        RETRO_TAPE(Constants.RETROTAPE_PIPELINE);

        private int id;
        Pipeline(int id) {
            this.id = id;
        }
    }

    public PipelineCommand(Vision vision, Pipeline pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight-vision").getEntry("pipeline").setNumber(pipeline.id);
    }

    @Override
    public boolean execute() {
        return true;
    }

    @Override
    public void shutdown() {
        
    }
    
}
