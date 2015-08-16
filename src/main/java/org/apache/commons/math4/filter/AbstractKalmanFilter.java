package org.apache.commons.math4.filter;

import org.apache.commons.math4.exception.DimensionMismatchException;
import org.apache.commons.math4.exception.NullArgumentException;
import org.apache.commons.math4.linear.CholeskyDecomposition;
import org.apache.commons.math4.linear.LUDecomposition;
import org.apache.commons.math4.linear.MatrixDimensionMismatchException;
import org.apache.commons.math4.linear.MatrixUtils;
import org.apache.commons.math4.linear.NonSquareMatrixException;
import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;
import org.apache.commons.math4.util.MathUtils;

public  abstract class AbstractKalmanFilter {
	
	
    /** The process model used by this filter instance. */
    private  ProcessModel processModel;
    /** The measurement model used by this filter instance. */
    private  MeasurementModel measurementModel;
    
	
    /** The internal state estimation vector, equivalent to x hat. */
    protected RealVector stateEstimation;
    /** The error covariance matrix, equivalent to P. */
    protected RealMatrix errorCovariance;
    
    

    
    
    
    public AbstractKalmanFilter(final ProcessModel process, final MeasurementModel measurement)
            throws NullArgumentException, NonSquareMatrixException, DimensionMismatchException,
            MatrixDimensionMismatchException 
    {
    	   MathUtils.checkNotNull(process);
           MathUtils.checkNotNull(measurement);

           this.processModel = process;
           this.measurementModel = measurement;
           
           
  

           

           
           errorCovariance = processModel.getInitialErrorCovariance();

           stateEstimation = processModel.getInitialStateEstimate();

    }
    
    /**
     * Returns the dimension of the state estimation vector.
     *
     * @return the state dimension
     */
    public int getStateDimension() {
        return stateEstimation.getDimension();
    }



    /**
     * Returns the current state estimation vector.
     *
     * @return the state estimation vector
     */
    public double[] getStateEstimation() {
        return stateEstimation.toArray();
    }

    /**
     * Returns a copy of the current state estimation vector.
     *
     * @return the state estimation vector
     */
    public RealVector getStateEstimationVector() {
        return stateEstimation.copy();
    }
    
    

    /**
     * Returns the current error covariance matrix.
     *
     * @return the error covariance matrix
     */
    public double[][] getErrorCovariance() {
        return errorCovariance.getData();
    }

    /**
     * Returns a copy of the current error covariance matrix.
     *
     * @return the error covariance matrix
     */
    public RealMatrix getErrorCovarianceMatrix() {
        return errorCovariance.copy();
    }
    
    
    
    protected void predict(RealMatrix transitionMatrix,RealVector u)
    {
    	
    	RealMatrix transitionMatrixT = transitionMatrix.transpose();
        // project the error covariance ahead
        // P(k)- = G * P(k-1) * G' + Q

    	RealMatrix s=transitionMatrix.multiply(errorCovariance);
    	RealMatrix ss = s.multiply(transitionMatrixT);
    	RealMatrix n = processModel.getProcessNoise(u);
    	
        errorCovariance = (transitionMatrix.multiply(errorCovariance))
                .multiply(transitionMatrixT)
                .add(processModel.getProcessNoise(u));
    }
    
    
    protected RealMatrix correct1( RealMatrix measurementMatrix)
    {
    	
    	RealMatrix measurementMatrixT = measurementMatrix.transpose();
        // S = H * P(k) * H' + R
        return measurementMatrix.multiply(errorCovariance)
            .multiply(measurementMatrixT)
            .add(measurementModel.getMeasurementNoise());
    }
    
    protected RealVector innovation(RealVector z ,RealVector w  )
    {
        // KF Inn = z(k) - H * xHat(k)-
        // EKF Inn = z(k) - h(mu)
    	
    	System.out.println("z vs w");
    	System.out.println("\tab w"+w);
    	System.out.println("\tab z"+z);
    	System.out.println("\tab z-w"+z.subtract(w));

        return  z.subtract(w);

    }
    
    
    protected RealMatrix kalmainGain(RealMatrix s, RealMatrix measurementMatrix )
    {
    	
        // calculate gain matrix
        // K(k) = P(k)- * H' * (H * P(k)- * H' + R)^-1
        // K(k) = P(k)- * H' * S^-1

        // instead of calculating the inverse of S we can rearrange the formula,
        // and then solve the linear equation A x X = B with A = S', X = K' and B = (H * P)'

        // K(k) * S = P(k)- * H'
        // S' * K(k)' = H * P(k)-'
    	
    	
    	
    	// KF 
    	// K(k) = P(k)- * H' * S^-1
    	
    	
        // K(k) * S = P(k)- * H'
        // S' * K(k)' = H * P(k)-'
    	
    	

    	
    	/*
    	 errorCovariance.multiply(measurementMatrix.transpose()).multiply( new LUDecomposition(s).getSolver().getInverse() )
    	  new CholeskyDecomposition(s).getSolver()
    	        .solve(measurementMatrix.multiply(errorCovariance.transpose()))
    	        .transpose();
    	            	System.out.println("cov " +errorCovariance.transpose() );
    	System.out.println("s " + s);
    	System.out.println("m "+ measurementMatrix);
    	System.out.println("m*cov"+measurementMatrix.multiply(errorCovariance.transpose()) );

    	*
    	*/

    	
       return    	 errorCovariance.multiply(measurementMatrix.transpose()).multiply( new LUDecomposition(s).getSolver().getInverse() );

	

    	
    }
    
    
    protected void correct2(RealMatrix kalmanGain, RealVector innovation)
    {
        // update estimate with measurement z(k)
        // xHat(k) = xHat(k)- + K * Inn
        stateEstimation = stateEstimation.add(kalmanGain.operate(innovation));

    }
    
    protected void correct3(RealMatrix kalmanGain, RealMatrix measurementMatrix )
    {
        // update covariance of prediction error
        // P(k) = (I - K * H) * P(k)-
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(kalmanGain.getRowDimension());
        errorCovariance = identity.subtract(kalmanGain.multiply(measurementMatrix)).multiply(errorCovariance);
        
    }
    
    

    
    

}
