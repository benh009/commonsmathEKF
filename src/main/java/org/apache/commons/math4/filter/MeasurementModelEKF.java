package org.apache.commons.math4.filter;

import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;

public interface MeasurementModelEKF extends MeasurementModel {

    /**
     * Returns the Jacobian H matrix.
     *
     * @return the Jacobian H  matrix
     */
    RealMatrix getJacobianH(int id);
    
    
    /**
     * Returns the value of h(mu) matrix.
     *
     * @return the value of h(mu) matrix
     */
    RealVector getStateEstimation( int id);
    
    
	
    void setFilter(ExtendedKalmanFilter filter) ;
	
	
}
