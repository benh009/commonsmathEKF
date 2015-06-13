package org.apache.commons.math4.filter;


import org.apache.commons.math4.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.linear.ArrayRealVector;
import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;
import org.junit.Before;
import org.junit.Test;

public class ExtendedKalmanFilterTest {

	@Before
	public void setUp() throws Exception {
	}


	
	
	  @Test
	    public void testFeatureBased() {
		  
		  
		  

	        // simulates a vehicle, Feature detected 



	    
	        // x = [ 0 0 0 ]
	        RealVector x = new ArrayRealVector(new double[] { 0, 0,0 });



	        // P0 = [ 1 1 1 ]
	        //      [ 1 1 1 ]
	        //      [ 1 1 1 ]
	        RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { { 1, 1,1 }, { 1, 1,1 }, { 1, 1,1 } });



	        // constant control input, increase velocity by 0.1 m/s per cycle
	        RealVector u = new ArrayRealVector(new double[] { 10.d,0.d });
	        
	        
	        final Double variance2ZD = 0.04;
	        final Double variance2D = 0.04;
	        final Double variance2S = 0.1;
	        final Double variance2Rho = 0.1 ;
	        final Double variance2Gamma = 0.1 ;
	        
	        
	        ProcessModelEKF pm = new DefaultProcessModelEKF( x,P0)  {
	        	
	        	


				@Override
	        	public RealVector getStateEstimation(RealVector u) {
					
	        		RealVector stateEstimation = filter.getStateEstimationVector();

	        		
	        		//  xt+1      =  x       +     d * cos (theta + gamma)
 	        		//  yt+1      =  y       +     d * sin (theta + gamma)
	        		//  thetat+1  =  theta   +     gamma
	        		
	        		Double rad = Math.toRadians(u.getEntry(1)+ stateEstimation.getEntry(2));
	        		Double d = u.getEntry(0);
	        		RealVector x = new ArrayRealVector(new double[] { 
	        				d*Math.cos(rad),
	        				d*Math.sin(rad),
	        				u.getEntry(1),
	        				});
	        		
	        		return stateEstimation.add(x);
	        	}

	        	@Override
	        	public RealMatrix getJacobianG(RealVector u) {
	        		RealVector stateEstimation = filter.getStateEstimationVector();
	        		
	        		
	        		Double rad = Math.toRadians(u.getEntry(1)+ stateEstimation.getEntry(2));
	        		
	        		Double d = u.getEntry(0);
	        		
	    	        RealMatrix G = new Array2DRowRealMatrix(new double[][] { 
	    	        		{ 1, 0, -d*Math.sin(rad) }, 
	    	        		{ 0, 1,  d*Math.cos(rad)    },
	    	        		{0,0,1 }
	    	        });

	        		
	        		
	        		return G;
	        	}

				@Override
				public RealMatrix getProcessNoise(RealVector u) {
					
					RealVector xVector = filter.getStateEstimationVector();

					Double rho = xVector.getEntry(2);
					Double d = u.getEntry(0);
					Double gamma = u.getEntry(1);
					Double radRhoGamma = Math.toRadians( rho + gamma) ;
					
					RealMatrix V = new Array2DRowRealMatrix(new double[][] { 
	    	        		{ Math.cos(radRhoGamma), -d * Math.sin(radRhoGamma) }, 
	    	        		{ Math.sin(radRhoGamma),d* Math.cos(radRhoGamma)    },
	    	        		{ 0, 1    }
	    	        }); 
					
					
					RealMatrix Vt = V.transpose();
					
					RealMatrix M = new Array2DRowRealMatrix(new double[][] { 
	    	        		{variance2D, 0 }, 
	    	        		{ 0, variance2Gamma}
	    	        }); 
					
					return V.multiply(M).multiply(Vt);
				}

				@Override
				public RealMatrix getProcessNoise() {
					// TODO Auto-generated method stub
					return null;
				}
	        	
	        };
	        
	        
	        
	        MeasurementModelEKF mm = new MeasurementModelEKF(){
	        	
	        	private ExtendedKalmanFilter filter ; 
	        	
	        	public void setFilter(ExtendedKalmanFilter filter) {
					this.filter = filter;
				}
	        	

	        	
	        	//BDD des features 
	        	
	        	double[][] features = new double[][]{
	        		{1.,2.,0.},
	        		{2.,3.,1.},
	        		{2.,0.,2.},
	        	};

				@Override
				public RealMatrix getJacobianH( int id) {
					
	        		RealVector stateEstimation = filter.getStateEstimationVector();

					double[] feature = features[id];
					double x = stateEstimation.getEntry(0);
					double y = stateEstimation.getEntry(1);
					
					double fx = feature[0];
					double fy = feature[1];
					
					double q = Math.pow(fx-x, 2)+ Math.pow(fy-y, 2);
					double rq = Math.sqrt(q);
					
	    	        RealMatrix H = new Array2DRowRealMatrix(new double[][] { 
	    	        		{-(fx-x) / rq,-(fy-y)/rq ,0}, 
	    	        		{(fy-y)/q,-(fx-x)/q,-1   },
	    	        		{0,0,0 }
	    	        });
					
					
					
					return H;
				}

				@Override
				public RealVector getStateEstimation(int id ) {
	        		RealVector stateEstimation = filter.getStateEstimationVector();

					double[] feature = features[id];
					double x = stateEstimation.getEntry(0);
					double y = stateEstimation.getEntry(1);
					double thehta = stateEstimation.getEntry(2);
					
					double fx = feature[0];
					double fy = feature[1];
					
	        		RealVector Zhat = new ArrayRealVector(new double[] { 
	        				Math.sqrt(  Math.pow(fx-x, 2)   +  Math.pow(fy-y,2) ),
	        				Math.atan2(fy-y, fx-x)-thehta,
	        				id

	        				});
					
					
					return  Zhat;
				}

				@Override
				public RealMatrix getMeasurementNoise() {
					
					
					
					
					
					RealMatrix Q = new Array2DRowRealMatrix(new double[][] { 
	    	        		{ variance2ZD, 0,0 }, 
	    	        		{ 0, variance2Rho, 0},
	    	        		{ 0,0 , variance2S}
	    	        }); ;; 
					
					
					return Q;
				}
	        	
	        };
	        ExtendedKalmanFilter filter = new ExtendedKalmanFilter(pm, mm);

	        
	         pm.setFilter( filter);
	         mm.setFilter(filter);
	         
	         



	        // iterate 60 steps
	        for (int i = 0; i < 60; i++) {
	            //filter.predict(u);
	            System.out.println("Predict : ");

	            System.out.println(filter.getStateEstimationVector());
	            System.out.println(filter.getErrorCovarianceMatrix());

	            

	            // z = H * x + m_noise
        		RealVector z = new ArrayRealVector(new double[] { 
        				1,0,2
        				});

	           filter.correct(z,2);
	           
	            System.out.println("correct : ");

	            System.out.println(filter.getStateEstimationVector());
	            System.out.println(filter.getErrorCovarianceMatrix());

	            


	        }


	    }
	  


}
