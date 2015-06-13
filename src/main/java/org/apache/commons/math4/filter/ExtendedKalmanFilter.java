package org.apache.commons.math4.filter;

import org.apache.commons.math4.exception.DimensionMismatchException;
import org.apache.commons.math4.exception.NullArgumentException;
import org.apache.commons.math4.linear.ArrayRealVector;
import org.apache.commons.math4.linear.MatrixDimensionMismatchException;
import org.apache.commons.math4.linear.NonSquareMatrixException;
import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;
import org.apache.commons.math4.linear.SingularMatrixException;

public class ExtendedKalmanFilter extends AbstractKalmanFilter {

    public ExtendedKalmanFilter(ProcessModelEKF process,
			MeasurementModelEKF measurement) throws NullArgumentException,
			NonSquareMatrixException, DimensionMismatchException,
			MatrixDimensionMismatchException {
		super(process, measurement);
		
		
        this.processModel = process;
        this.measurementModel = measurement;
        

	}

	/** The process model used by this filter instance. */
    private final ProcessModelEKF processModel;
    /** The measurement model used by this filter instance. */
    private final MeasurementModelEKF measurementModel;
	
	
	
	 
    /**
     * Predict the internal state estimation one time step ahead.
     */
    public void predict() {
        predict((RealVector) null);
    }

    /**
     * Predict the internal state estimation one time step ahead.
     *
     * @param u
     *            the control vector
     * @throws DimensionMismatchException
     *             if the dimension of the control vector does not fit
     */
    public void predict(final double[] u) throws DimensionMismatchException {
        predict(new ArrayRealVector(u, false));
    }

    /**
     * Predict the internal state estimation one time step ahead.
     *
     * @param u
     *            the control vector
     * @throws DimensionMismatchException
     *             if the dimension of the control vector does not match
     */
    public void predict(final RealVector u) throws DimensionMismatchException {

        stateEstimation = processModel.getStateEstimation(u);
        RealMatrix g = processModel.getJacobianG(u);
        super.predict(g,u);

    }
	
    /**
     * Correct the current state estimate with an actual measurement.
     *
     * @param z
     *            the measurement vector
     * @throws NullArgumentException
     *             if the measurement vector is {@code null}
     * @throws DimensionMismatchException
     *             if the dimension of the measurement vector does not fit
     * @throws SingularMatrixException
     *             if the covariance matrix could not be inverted
     */
    public void correct(final double[] z,int id)
            throws NullArgumentException, DimensionMismatchException, SingularMatrixException {
        correct(new ArrayRealVector(z, false),id);
    }

    /**
     * Correct the current state estimate with an actual measurement.
     *
     * @param z
     *            the measurement vector
     * @throws NullArgumentException
     *             if the measurement vector is {@code null}
     * @throws DimensionMismatchException
     *             if the dimension of the measurement vector does not fit
     * @throws SingularMatrixException
     *             if the covariance matrix could not be inverted
     */
    public void correct(final RealVector z,int id)
            throws NullArgumentException, DimensionMismatchException, SingularMatrixException {

    	RealMatrix H = measurementModel.getJacobianH( id);
    	

        
     // S = H * P(k) * H' + R
        RealMatrix s = super.correct1(H);

        // Inn = z(k) - H * xHat(k)-
        
       
        RealVector innovation = super.innovation(z, measurementModel.getStateEstimation( id));

        // calculate gain matrix
        // K(k) = P(k)- * H' * (H * P(k)- * H' + R)^-1
        // K(k) = P(k)- * H' * S^-1

        // instead of calculating the inverse of S we can rearrange the formula,
        // and then solve the linear equation A x X = B with A = S', X = K' and B = (H * P)'

        // K(k) * S = P(k)- * H'
        // S' * K(k)' = H * P(k)-'
        RealMatrix kalmanGain = super.kalmainGain( s,H);

        
        // update estimate with measurement z(k)
        // xHat(k) = xHat(k)- + K * Inn
        super.correct2(kalmanGain, innovation);
        
        // update covariance of prediction error
        // P(k) = (I - K * H) * P(k)-
  
        super.correct3(kalmanGain,H);
        
    }
	
}
