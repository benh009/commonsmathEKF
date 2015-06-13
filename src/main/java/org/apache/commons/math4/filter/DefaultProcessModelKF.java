package org.apache.commons.math4.filter;

import org.apache.commons.math4.exception.DimensionMismatchException;
import org.apache.commons.math4.exception.NoDataException;
import org.apache.commons.math4.exception.NullArgumentException;
import org.apache.commons.math4.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.linear.ArrayRealVector;
import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;

public class DefaultProcessModelKF extends DefaultProcessModel implements
		ProcessModelKF {

    /**
     * The control matrix, used to integrate a control input into the state estimation.
     */
    private final RealMatrix controlMatrix;

    /**
     * The state transition matrix, used to advance the internal state estimation each time-step.
     */
    private final RealMatrix stateTransitionMatrix;

	
	/** The process noise covariance matrix. */
    protected final RealMatrix processNoiseCovMatrix;

    
	
	
	   /**
     * Create a new {@link ProcessModel}, taking double arrays as input parameters.
     *
     * @param stateTransition
     *            the state transition matrix
     * @param control
     *            the control matrix
     * @param processNoise
     *            the process noise matrix
     * @param initialStateEstimate
     *            the initial state estimate vector
     * @param initialErrorCovariance
     *            the initial error covariance matrix
     * @throws NullArgumentException
     *             if any of the input arrays is {@code null}
     * @throws NoDataException
     *             if any row / column dimension of the input matrices is zero
     * @throws DimensionMismatchException
     *             if any of the input matrices is non-rectangular
     */
    public DefaultProcessModelKF(final double[][] stateTransition,
                               final double[][] control,
                               final double[][] processNoise,
                               final double[] initialStateEstimate,
                               final double[][] initialErrorCovariance)
            throws NullArgumentException, NoDataException, DimensionMismatchException {

        this(new Array2DRowRealMatrix(stateTransition),
                new Array2DRowRealMatrix(control),
                new Array2DRowRealMatrix(processNoise),
                new ArrayRealVector(initialStateEstimate),
                new Array2DRowRealMatrix(initialErrorCovariance));
    }

    /**
     * Create a new {@link ProcessModel}, taking double arrays as input parameters.
     * <p>
     * The initial state estimate and error covariance are omitted and will be initialized by the
     * {@link KalmanFilter} to default values.
     *
     * @param stateTransition
     *            the state transition matrix
     * @param control
     *            the control matrix
     * @param processNoise
     *            the process noise matrix
     * @throws NullArgumentException
     *             if any of the input arrays is {@code null}
     * @throws NoDataException
     *             if any row / column dimension of the input matrices is zero
     * @throws DimensionMismatchException
     *             if any of the input matrices is non-rectangular
     */
    public DefaultProcessModelKF(final double[][] stateTransition,
                               final double[][] control,
                               final double[][] processNoise)
            throws NullArgumentException, NoDataException, DimensionMismatchException {

        this(new Array2DRowRealMatrix(stateTransition),
                new Array2DRowRealMatrix(control),
                new Array2DRowRealMatrix(processNoise), null, null);
    }

    /**
     * Create a new {@link ProcessModel}, taking double arrays as input parameters.
     *
     * @param stateTransition
     *            the state transition matrix
     * @param control
     *            the control matrix
     * @param processNoise
     *            the process noise matrix
     * @param initialStateEstimate
     *            the initial state estimate vector
     * @param initialErrorCovariance
     *            the initial error covariance matrix
     */
    public DefaultProcessModelKF(final RealMatrix stateTransition,
                               final RealMatrix control,
                               final RealMatrix processNoise,
                               final RealVector initialStateEstimate,
                               final RealMatrix initialErrorCovariance) {
    	super(initialStateEstimate,initialErrorCovariance);
    	
        this.processNoiseCovMatrix = processNoise;

        this.controlMatrix = control;
        this.stateTransitionMatrix = stateTransition;


    }
	
	
	



    /** {@inheritDoc} */
    @Override
    public RealMatrix getControlMatrix() {
        return controlMatrix;
    }
    
    /** {@inheritDoc} */
    @Override
    public RealMatrix getStateTransitionMatrix() {
        return stateTransitionMatrix;
    }
    /** {@inheritDoc} */
    @Override
    public RealMatrix getProcessNoise(RealVector u) {
        return processNoiseCovMatrix;
    }
    
    /** {@inheritDoc} */
    @Override
    public RealMatrix getProcessNoise() {
        return processNoiseCovMatrix;
    }
    

}
