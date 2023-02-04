package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

public class BeamBreak {
    
	private final DigitalInput mBeamInput;
	private FilteredAverage mFilter = null;
	
	public BeamBreak(int pInputChannel) {

		mBeamInput = new DigitalInput(pInputChannel);

	}

	public BeamBreak(int pInputChannel, double pDebounceTime) {
		int numFilters = (int)(pDebounceTime / 0.02);
		double[] gains = new double[numFilters];
		Arrays.fill(gains, 1.0/(double)numFilters);
		mFilter = new FilteredAverage(gains);
		mBeamInput = new DigitalInput(pInputChannel);
	}

	public boolean isBroken() {
	  // NOTE - if the beam is noisy, we can do some filtered average based upon the leading
	  // edge of the detection rather than just a 'get'.  This effectively debounces the signal.
	  //	  mBeamInput.readRisingTimestamp()
		// If a debounce time was requested
		if(mFilter != null) {
			mFilter.addNumber(mBeamInput.get() ? 1.0 :0.0);
			SmartDashboard.putNumber("average: ", mFilter.getAverage());
			SmartDashboard.putNumber("num added: ", mBeamInput.get() ? 1.0 : 0.0);
			return mFilter.getAverage() <= 0.5;
		} else {
			return mBeamInput.get();
		}
	}

	public boolean get() {
		return mBeamInput.get();
	}
	
}
