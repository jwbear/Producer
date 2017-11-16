package com.kafka.producer;

import java.io.IOException;



public class Produce 
{
	//public static ProducerSimConnect sim = new ProducerSimConnect();
	private static final int RUN_TYPE = 2; 
	
	 public static void main(String[] args) throws IOException {
		 
		 if(RUN_TYPE==0){	//file run
			 ProducerFile fileTest = new ProducerFile();
			
		  }else if(RUN_TYPE==1){	//sim run
			  ProducerSimConnect sim = new ProducerSimConnect();
			  sim.slam1();
			  //start threads
			  ProducerSim simTest = new ProducerSim(sim);
			  //test slam
			  for(int t = 0; t < 20; t++){ 
				  sim.slam2();
				  sim.slam1();
				  
			  }sim.getMap();
			  System.exit(1);
		  }else if(RUN_TYPE==2){	//kafka streaming run, uses 2 ml threads
			  ProducerSimConnect sim = new ProducerSimConnect();
			  //init vars
			  sim.slam1();
			  //start threads
			  ProducerSim simTest = new ProducerSim(sim);
			  while(true){
			  //init vars
			  sim.slam2();
			  sim.slam1();
			  }
		  }
		
	    }
	 

}
