package org.apache.commons.math4.filter;


import org.apache.commons.math4.linear.Array2DRowRealMatrix;
import org.apache.commons.math4.linear.ArrayRealVector;
import org.apache.commons.math4.linear.MatrixUtils;
import org.apache.commons.math4.linear.RealMatrix;
import org.apache.commons.math4.linear.RealVector;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

public class ExtendedKalmanFilterTest {

	@Before
	public void setUp() throws Exception {
		
		  // simulates a vehicle, Feature detected 


		//0.0001 = 1cm  

	    
        // x = [ 0 10 0 ]
        RealVector x = new ArrayRealVector(new double[] { 0, 0,Math.PI/4 });



        // P0 = [ 1 1 1 ]
        //      [ 1 1 1 ]
        //      [ 1 1 1 ]
        RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { {10, 0,0 }, { 0, 10,0 }, { 0,0,Math.PI/2} });




        
        final Double variance2ZD = 1.;
        //0.01
        final Double variance2D = 0.001;

        final Double variance2Rho = 1. ;
        final Double variance2Gamma = 0.01 ;
        
        
        ProcessModelEKF pm = new DefaultProcessModelEKF( x,P0)  {
        	
        	


			@Override
        	public RealVector getStateEstimation(RealVector u) {
				//u en degre qui est trans en rad 
				
        		RealVector stateEstimation = filter.getStateEstimationVector();

        		
        		//  xt+1      =  x       +     d * cos (theta )
	        		//  yt+1      =  y       +     d * sin (theta )
        		//  thetat+1  =  theta   +     gamma

        		Double rad = u.getEntry(1)+stateEstimation.getEntry(2);
        		Double d = u.getEntry(0);
        		RealVector x = new ArrayRealVector(new double[] { 
        				d*Math.cos(rad),
        				d*Math.sin(rad),
        				u.getEntry(1)%(Math.PI*2),
        				});
        		
        		return stateEstimation.add(x);
        	}

        	@Override
        	public RealMatrix getJacobianG(RealVector u) {
        		RealVector stateEstimation = filter.getStateEstimationVector();
        		
        		
        		Double rad = stateEstimation.getEntry(2);
        		
        		Double d = u.getEntry(0);
        		Double gamma = u.getEntry(1);
        		
    	        RealMatrix G = new Array2DRowRealMatrix(new double[][] { 
    	        		{ 1, 0, -d*Math.sin(rad+gamma) }, 
    	        		{ 0, 1,  d*Math.cos(rad+gamma)    },
    	        		{0,0,1 }
    	        });

        		
        		return G;
        	}

			@Override
			public RealMatrix getProcessNoise(RealVector u) {
				
				RealVector xVector = filter.getStateEstimationVector();

				Double radRhoGamma  = xVector.getEntry(2);
				Double d = u.getEntry(0);
				Double angle = u.getEntry(1);
				
				Double aD = Math.abs(d);
				
				Double aAngle = Math.abs(angle);
				
				Double alpha1 = 0.001 ;
				Double alpha2 = 0.01;
				Double alpha3 = 0.01;
				Double alpha4 = 0.001;
				Double errorTrans = alpha1 * aAngle + alpha2 * aD ;
				Double errorRot = alpha3 * aAngle + alpha4 *aD;
				
				RealMatrix V = new Array2DRowRealMatrix(new double[][] { 
    	        		{ Math.cos(radRhoGamma), -d * Math.sin(radRhoGamma) }, 
    	        		{ Math.sin(radRhoGamma),d* Math.cos(radRhoGamma)    },
    	        		{ 0, 1    }
    	        }); 
				
				
				RealMatrix Vt = V.transpose();
				
				RealMatrix M = new Array2DRowRealMatrix(new double[][] { 
    	        		{errorTrans, 0 }, 
    	        		{ 0,errorRot}
    	        }); 

				RealMatrix res = V.multiply(M).multiply(Vt);

				//MatrixUtils.checkSymmetric(res, 0.1);
				
				return res;
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
        		{50.,0.}};

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
				
				System.out.println("q distance "+rq);

				
    	        RealMatrix H = new Array2DRowRealMatrix(new double[][] { 
    	        		{-(fx-x) / rq,-(fy-y)/rq ,0}, 
    	        		{(fy-y)/q,-(fx-x)/q,-1   }
    	        		
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
				
				System.out.println("angle" +  (thehta));
				
        		RealVector Zhat = new ArrayRealVector(new double[] { 
        				Math.sqrt(  Math.pow(fx-x, 2)   +  Math.pow(fy-y,2) ),
        				(Math.atan2(fy-y, fx-x)-thehta)%(Math.PI*2),
        				});
				
				
				return  Zhat;
			}

			@Override
			public RealMatrix getMeasurementNoise() {
				
				
				
				
				
				RealMatrix Q = new Array2DRowRealMatrix(new double[][] { 
    	        		{ variance2ZD, 0 }, 
    	        		{ 0, variance2Rho},
    	        		
    	        }); 
				
				
				return Q;
			}
        	
        };
        ExtendedKalmanFilter filter = new ExtendedKalmanFilter(pm, mm);

        
         pm.setFilter( filter);
         mm.setFilter(filter);
		
         this.filter = filter;
		
	}


	public ExtendedKalmanFilter filter ;
	
	@Ignore
	 @Test
	  public void test0( )
	  {
		  printName("Test0");



	         RealVector u = new ArrayRealVector(new double[] {0.01d,0.d });

	         
	         filter.predict(u);
	         filter.predict(u);

	         filter.predict(u);
	         filter.predict(u);




	         System.out.println("Predict : ");
	         System.out.println(filter.getStateEstimationVector());
	         System.out.println(filter.getErrorCovarianceMatrix());

	         

		  

	  }
	
/**
 * 0  O(10,0)        x(50,0)
 * 
 * 
 * 1    O(20,0)      x(50,0)
 * 
 * 2        O(30,0) x(50,0)
 * 
 * 3          O(40,0) x(50,0)
 * 
 */
	


		@Ignore
	  @Test
	  public void test1( )
	  {
		  printName("Test1");

		  RealVector z;

		  
	         RealVector u = new ArrayRealVector(new double[] { 0,-44 });
	         z = new ArrayRealVector(new double[] { 50.d,0});
	         predictCorrect(filter, u,z,0);
		  	

	         u = new ArrayRealVector(new double[] { 10.d,0 });
	         z = new ArrayRealVector(new double[] { 40.d,0.});
	         predictCorrect(filter, u,z,0);
	         
	         
	         u = new ArrayRealVector(new double[] { 10.d,0.d });
	         z = new ArrayRealVector(new double[] { 25.d,0.});
	         predictCorrect(filter, u,z,0);
	         
	         
	         u = new ArrayRealVector(new double[] { 10.d,0.d });
	         z = new ArrayRealVector(new double[] { 10.d,0.});
	         predictCorrect(filter, u,z,0);


		  

	  }
	  /**
	   * 0 O(0,0,0)        x(50,0)
	   * 1 O(0,10,90)       x(50,0)
	   * 2                  x(50,0)
	   *   O(10,10,90)
	   * 3                  x(50,0)
	   *   O(10,10,11)
	   * 
	   *     
	   */
	@Ignore
	  @Test
	  public void test2()
	  {
		  printName("Test2");

		  
		  RealVector z;

		  	
	         RealVector u = new ArrayRealVector(new double[] { 0.d,0.d});
	         z = new ArrayRealVector(new double[] { 50.d,-Math.PI/2});
	         predictCorrect(filter, u,z,0);
	         
	         
	         
	         u = new ArrayRealVector(new double[] { 10.d,0.d });
	         z = new ArrayRealVector(new double[] { 50.99d,-Math.toRadians(101.31)});
	         predictCorrect(filter, u,z,0);
	         
	         
	         u = new ArrayRealVector(new double[] { 0.d,-101.31d });
	         z = new ArrayRealVector(new double[] { 50.99d,0.});
	         predictCorrect(filter, u,z,0);
	         
	         
	         u = new ArrayRealVector(new double[] { 30.d,0d });
	         z = new ArrayRealVector(new double[] { 10d,0.});
	         predictCorrect(filter, u,z,0);


		  

	  }
	  //@Ignore
	  @Test
	  public void test3()
	  {
		  printName("Test3");
		  
	         RealVector u = new ArrayRealVector(new double[] {0d,Math.toRadians(40) });

	         predict( u);
	         
	          u = new ArrayRealVector(new double[] {200d,0 });
		      predict( u);
	         
	         

	  }
	  
	  
	  @Test
	  public void test4()
	  {
		  printName("Test4");
		  
	         RealVector u = new ArrayRealVector(new double[] {0d,Math.toRadians(5) });
	         
	         predict( u);
	         predict( u);
	         predict( u);
	         predict( u);
	         predict( u);
	         predict( u);
	         predict( u);
	         predict( u);
	         
	          u = new ArrayRealVector(new double[] {1d,0 });
		      predict( u);

	          u = new ArrayRealVector(new double[] {0d,Math.toRadians(180)  });

		      predict( u);
		      
	          u = new ArrayRealVector(new double[] {10d,0 });
		      predict( u);


	  }
	  
	  
	  public void predict(RealVector u ){
	         filter.predict(u);
	         System.out.println("Predict u : "+ "d "+u.getEntry(0)+" a "+ u.getEntry(1));
	         System.out.println("\t"+filter.getStateEstimationVector());
	         System.out.println("\t"+filter.getErrorCovarianceMatrix());
	         MatrixUtils.checkSymmetric(filter.getErrorCovarianceMatrix(), 0.1);	


	  }
	  
	  
	  public void predictCorrect( ExtendedKalmanFilter filter,RealVector u ,RealVector z, int idZ   )
	  {

		  predict(  u );

	         // z = H * x + m_noise
	         filter.correct(z,idZ);
	         System.out.println("correct : ");
	         System.out.println(filter.getStateEstimationVector());
	         System.out.println(filter.getErrorCovarianceMatrix());
	         
	         System.out.println("--------------------------------");
				//MatrixUtils.checkSymmetric(filter.getErrorCovarianceMatrix(), 0.1);	


	  }
	  
	  
	  public void printName(String NameTest)
	  {
		  
	         System.out.println();

	         System.out.println();
	         System.out.println("--------------------------------");
	         System.out.println(NameTest);
	         
	         System.out.println("--------------------------------");

	         System.out.println();

	  }
	  
	  
	  
	  


}
