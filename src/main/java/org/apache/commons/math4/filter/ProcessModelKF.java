package org.apache.commons.math4.filter;

import org.apache.commons.math4.linear.RealMatrix;

public interface ProcessModelKF extends ProcessModel {
	

    /**
     * Returns the control matrix.
     *
     * @return the control matrix
     */
    RealMatrix getControlMatrix();
    
    
    /**
     * Returns the state transition matrix.
     *
     * @return the state transition matrix
     */
    RealMatrix getStateTransitionMatrix();


}
