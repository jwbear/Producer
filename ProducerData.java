package com.kafka.producer;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/*
 * Get simulated raw data from file in simulated real time
 * for Kafka Producer
 * 
 * stamp.nsec: nanoseconds since stamp_secs 
 */

public class ProducerData {

	
	/*
	 * Get data from file in simulated real time
	 */
	public BufferedReader getOdometryData(int trial){		
		 // The name of the file to open.
        String fileName = "/Users/Timshel/Documents/workspace/"
        		+ "produce/data/input/odometry_"+trial+".txt";

        try {
            // FileReader reads text files in the default encoding.
            FileReader fileReader = 
                new FileReader(fileName);
        
            // Always wrap FileReader in BufferedReader.
            BufferedReader bufferedReader = 
                new BufferedReader(fileReader);

           return bufferedReader;    
           
        } catch(FileNotFoundException ex) {
        	ex.printStackTrace();
            System.out.println(
                "Unable to open file '" + 
                fileName + "'"); 
        }
        catch(IOException ex) {
            System.out.println(
                "Error reading file '" 
                + fileName + "'");                  
            // Or we could just do this: 
            // ex.printStackTrace();
        }
		return null;
	}    
	
	
	
	/*
	 * Get data from file in simulated real time
	 */
	public BufferedReader getLaserData(int trial){
		 // The name of the file to open.
        String fileName = "/Users/Timshel/Documents/workspace/"
        		+ "produce/data/input/laserData_"+trial+".txt";

        try {
            // FileReader reads text files in the default encoding.
            FileReader fileReader = 
                new FileReader(fileName);
        
            // Always wrap FileReader in BufferedReader.
            BufferedReader bufferedReader = 
                new BufferedReader(fileReader);

          return bufferedReader;
          
        }catch(FileNotFoundException ex) {
            System.out.println(
                "Unable to open file '" + 
                fileName + "'");                
        }
        catch(IOException ex) {
            System.out.println(
                "Error reading file '" 
                + fileName + "'");                  
            // Or we could just do this: 
            // ex.printStackTrace();
        }
        
        return null;
	}    
	
	public BufferedReader getLaserRanges(int trial){		
		 // The name of the file to open.
        String fileName = "/Users/Timshel/Documents/workspace/"
        		+ "produce/data/input/laserRanges_"+trial+".txt";

        try {
            // FileReader reads text files in the default encoding.
            FileReader fileReader = 
                new FileReader(fileName);
        
            // Always wrap FileReader in BufferedReader.
            BufferedReader bufferedReader = 
                new BufferedReader(fileReader);

         return bufferedReader;
         
        }
        catch(FileNotFoundException ex) {
            System.out.println(
                "Unable to open file '" + 
                fileName + "'");                
        }
        catch(IOException ex) {
            System.out.println(
                "Error reading file '" 
                + fileName + "'");                  
            // Or we could just do this: 
            // ex.printStackTrace();
        }
		return null;
	}   
	

}

	
