package com.kafka.producer;

import org.apache.kafka.clients.producer.Producer;
import org.apache.kafka.clients.producer.ProducerRecord;

public class ProducerSim {
	
	public ProducerSim(ProducerSimConnect sim){	
		try {

	        	 //publish odometry data 
	        	 Thread threadOdom = new Thread("threadOdomSim") {
	        		 
	        	      public void run(){
	        	    	  //instance must be created inside run class
	        	    	  ProducerClass prod = new ProducerClass();
	        	    	  Producer<String,String> prodr = prod.getProducer();
	        	    	  
	     	        	 //get odom file
	        	    	  try {
							while(true){
								double[] odom = sim.getOdom();
								String odomLine = 
										odom[1] +","+ odom[2] + ","+ odom[3];
							    prodr.send(new ProducerRecord<String, String>
							    	("odom", String.valueOf(odom[0]),odomLine));
							     prodr.flush();
							     System.out.println("odom " + odomLine);
							     //laser scanner intervals
							    Thread.sleep(1000);
							}				
							
						} catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
	        	    	  
	        	     }
	        	 };
	        	 
	        	 //publish odometry data 
	        	 Thread threadLaser = new Thread("threadLaserSim") {
	        	      public void run(){
	        	    	  //instance must be new in each thread to maintain thread safety
	        	    	  ProducerClass prod = new ProducerClass();
	        	    	  Producer<String,String> prodr = prod.getProducer();
	     	        	
	     	        	 
	     	        	 String laserLine = ""; 
	        	    	  try {
							while(true){
								//get laser data
								float[][] laser = sim.getLaser();
								//parse data to time, x, y, ranges
								//read first entry
								String laserRanges = "";
								//row
								for(int i = 0; i<616; i++ ){
									String ranges ="[";
									//get ranges
									for(int m = 3; m < 640; m++){
										ranges += laser[i][m] + ",";
									}ranges += "]";
									//col
									
									laserRanges = laser[i][1] +","+ 
											laser[i][2] +","+ ranges; 
								
								System.out.println("laserline " + laserRanges);
						    	 prodr.send(
						    			 new ProducerRecord<String, String>
						    			 ("laser",String.valueOf(laser[i][0]), laserRanges));
						    	 prodr.flush();
						    	 //movement per sec
							    Thread.sleep(45);
								}
							}
							
						} catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
	        	      }
	        	 };
	        	 
	        	 threadOdom.start();
	        	 threadLaser.start();
	     
 
	            } catch (Exception e) {
	            	System.out.println(e);
	            	System.out.println(e.getStackTrace());
	            	
		        } catch (Throwable throwable) {
		        	System.out.println("H" + throwable.getLocalizedMessage());
	            	System.out.println(throwable.getStackTrace());
	            	
		        } finally {
		        	ProducerClass prod = new ProducerClass();
		        	prod.closeProducer();
		        }
	    }
	}


