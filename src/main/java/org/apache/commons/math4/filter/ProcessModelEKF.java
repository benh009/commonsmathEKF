package org.apache.commons.math4.filter;

import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;
/**
 * @author benoitH
 *
 */
public interface ProcessModelEKF extends ProcessModel {

	
    /**
     * Returns the stateEstimation matrix(value of g(u,mu)).
     *
     * @return the stateEstimation matrix
     */
    RealVector getStateEstimation(final RealVector u);
    
    
	
    /**
     * Returns the Jacobian G matrix.
     *
     * @return the Jacobian G  matrix
     */
    RealMatrix getJacobianG(RealVector u);



	void setFilter(ExtendedKalmanFilter filter);
    
    
    




}
