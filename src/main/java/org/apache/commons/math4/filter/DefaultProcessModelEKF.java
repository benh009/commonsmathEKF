package org.apache.commons.math4.filter;

import org.apache.commons.math4.exception.DimensionMismatchException;
import org.apache.commons.math4.exception.NoDataException;
import org.apache.commons.math4.exception.NullArgumentException;
import org.apache.commons.math4.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.linear.ArrayRealVector;
import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;
/**
 * @author benoitH
 *
 */
public abstract class DefaultProcessModelEKF extends DefaultProcessModel implements
		ProcessModelEKF {
	
	
	
	protected ExtendedKalmanFilter filter ; 
	
	public void setFilter(ExtendedKalmanFilter filter) {
		this.filter = filter;
	}

	
	   /**
  * Create a new {@link ProcessModel}, taking double arrays as input parameters.
  *
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
 public DefaultProcessModelEKF(
                            final double[] initialStateEstimate,
                            final double[][] initialErrorCovariance)
         throws NullArgumentException, NoDataException, DimensionMismatchException {

     this(
             
             new ArrayRealVector(initialStateEstimate),
             new Array2DRowRealMatrix(initialErrorCovariance));
 }



 /**
  * Create a new {@link ProcessModel}, taking double arrays as input parameters.
  *
  * @param stateTransition
  *            the state transition matrix
  * @param initialStateEstimate
  *            the initial state estimate vector
  * @param initialErrorCovariance
  *            the initial error covariance matrix
  */
 public DefaultProcessModelEKF(
                            
                            final RealVector initialStateEstimate,
                            final RealMatrix initialErrorCovariance) {
 	super(initialStateEstimate,initialErrorCovariance);



 }



}
