package com.kafka.producer;
/*
 * Create initial producer and share instance
 * as recommended by Kafka
 */
import java.io.BufferedReader;
import java.io.IOException;
import java.util.Properties;

import org.apache.kafka.clients.producer.KafkaProducer;
import org.apache.kafka.clients.producer.Producer;
import org.apache.kafka.clients.producer.ProducerRecord;


public class ProducerClass{
	
	Producer<String, String> producer;
	
    public ProducerClass(){
    	 Properties props = new Properties();
    	 //list of servers/main node for cluster
    	 props.put("bootstrap.servers", "localhost:9092");
    	 //acknowledgment from consumers
    	 props.put("acks", "0");
    	 //not allowed to prevent ordering change
    	 props.put("retries", 0);
    	 //maximum batch sent to a partition
    	 //props.put("batch.size", 16384);
    	 props.put("batch.size", 0);
    	 //do not add latency waiting for batch: adjust??
    	 props.put("linger.ms", 0);
    	 //max buffer size
    	 props.put("buffer.memory", 33554432);
    	 //serialization
    	   props.put("key.serializer", "org.apache.kafka.common.serialization.StringSerializer");
    	   props.put("value.serializer", "org.apache.kafka.common.serialization.StringSerializer");
    	   //fixes bug were metadata hands send
    	   props.put("metadata.fetch.timeout.ms", 1000);
    	 producer = new KafkaProducer<>(props);
    	 
    }

	 
	 public Producer<String, String> getProducer(){
		 return this.producer;
	 }
	 
	 public void closeProducer(){
		 this.producer.close();
	 }

}
