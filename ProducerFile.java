package com.kafka.producer;

import java.io.BufferedReader;
import java.io.IOException;

import org.apache.kafka.clients.producer.Producer;
import org.apache.kafka.clients.producer.ProducerRecord;

public class ProducerFile {
	
	public ProducerFile(){
   	  	 ProducerData data = new ProducerData();
		 
	        	 try {
	        		 
	        	 //Kafka File IO and publish
	        	 
	        	 
	        	 //publish odometry data 
	        	 Thread threadOdom = new Thread("threadOdom") {
	        		 
	        	      public void run(){
	        	    	  //instance must be created inside run class
	        	    	  ProducerClass prod = new ProducerClass();
	        	    	  Producer<String,String> prodr = prod.getProducer();
	        	    	  
	     	        	 //get odom file
	     	        	 BufferedReader readOdom = data.getOdometryData(0);
	     	        	 String odomLine; 
	        	    	  try {
							while((odomLine = readOdom.readLine()) != null){
								System.out.println(odomLine);
							    prodr.send(new ProducerRecord<String, String>
							    	("odom", 
							    	String.valueOf(System.currentTimeMillis()),
							    	odomLine));
							     prodr.flush();
							    Thread.sleep(1000);
							}
							
							readOdom.close();
							
							
						} catch (IOException | InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
	        	    	  
	        	     }
	        	 };
	        	 
	        	 //publish odometry data 
	        	 Thread threadLaser = new Thread("threadLaser") {
	        	      public void run(){
	        	    	  //instance must be new in each thread to maintain thread safety
	        	    	  ProducerClass prod = new ProducerClass();
	        	    	  Producer<String,String> prodr = prod.getProducer();
	     	        	 //get odom file
	     	        	 BufferedReader readLaser = data.getLaserData(0);
	     	        	BufferedReader readLaserRanges = data.getLaserRanges(0);
	     	        	 
	     	        	 String laserLine = ""; 
	        	    	  try {
							while(laserLine != null){
								//read first entry
								String laserRanges = readLaserRanges.readLine();
								for(int i = 0; i<640; i++ ){
									String tmp = readLaserRanges.readLine();
									if(tmp == null)
										break;
									laserRanges = laserRanges + "," + tmp;
								}//System.out.println("ranges " + laserRanges);
								laserLine = "";
								for(int i = 0; i<616; i++ ){
									String tmp = readLaser.readLine();
									if(tmp == null)
										break;
									laserLine = laserLine + " " + tmp;
								}
								//dont broadcast blank lines
								if(laserLine.equals(null) || laserLine.length()<6) 
									break;
							    laserLine = laserLine + " [" + laserRanges + "]";	
								System.out.println("laserline" + laserLine);
							    	 prodr.send(
							    			 new ProducerRecord<String, String>
							    			 ("laser",String.valueOf(System.currentTimeMillis()), laserLine));
							    	 prodr.flush();
							    Thread.sleep(30);
							}
							
							readLaser.close();
							readLaserRanges.close();
							
						} catch (IOException | InterruptedException e) {
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


