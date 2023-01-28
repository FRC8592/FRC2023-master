package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;


public class FilteredAverage {
  private final ArrayList<Double> mNumbers = new ArrayList<Double>();
  private final double[] mGains;

    /**
     * @param pGains - applied to the average. First number in
     *               the list is applied to the oldest number in the average. Running average length matches the length
     *               of the gains.
     */
  public FilteredAverage(double[] pGains) {
    if(pGains == null || pGains.length == 0) {
      throw new IllegalArgumentException("Cannot create a filtered average without filter gains!");
    }
    mGains = pGains;
  }

  public void addNumber(double newNumber) {
      mNumbers.add(newNumber);
      if (!isUnderMaxSize()) {
          mNumbers.remove(0);
      }
    }

  public double getAverage() {
      double result = 0;
      
      int g = 0;
      for (int i = mNumbers.size()-1; i >= 0; i--) {
        if(g >= 0 && g < mGains.length) {
          result += mGains[g] * mNumbers.get(i);
          g++;
        }
        
      }

      return result;
  }

  public int getSize() {
      return mNumbers.size();
  }

  public boolean isUnderMaxSize() {
      return getSize() <= mGains.length;
  }

  public void clear() {
      mNumbers.clear();
  }

    /**
     * @param pTime in seconds
     * @return a running average that should contains the last pTime seconds' worth of data
     */
  public static FilteredAverage filteredAverageForTime(double pTime) {
      int numCycles = (int)(pTime / 0.02);
      double[] gains = new double[numCycles];
      Arrays.fill(gains, 1.0 / (double)numCycles);
      return new FilteredAverage(gains);
  }
}
