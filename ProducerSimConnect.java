package com.kafka.producer;

import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;
import matlabcontrol.MatlabProxyFactoryOptions;

public class ProducerSimConnect {

	MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder()
		    .setUsePreviouslyControlledSession(true)
		    //.setHidden(true)
		    .setMatlabLocation(null).build();
	
	//Create a proxy, which we will use to control MATLAB 
	MatlabProxyFactory factory = new MatlabProxyFactory();
	MatlabProxy proxy;

	
	private static final String slam0 = "tic;\n" +
			  "while toc < 20\n";
	
	private static final String slam1 = 
		      "% Collect information from laser scan\n"+
		      "scan = receive(laser);\n"+
		      "data = readCartesian(scan);\n"+
		      "angle = readScanAngles(scan);\n"+
		      "x = data(:,1);\n"+
		      "y = data(:,2);\n"+
		      "laserRanges(:,1) = scan.Ranges;\n"+
		      "laserRanges(:,2) = angle(:,1);\n"+
		      "odomdata = receive(odom);\n" +
		      "ot = odomdata.Header.Seq;\n" +
		      "ol = scan.Header.Seq;\n" +
				"pose = odomdata.Pose.Pose;\n"+
				"ox = pose.Position.X; \n"+
				"oy = pose.Position.Y;\n"+
				"quat = pose.Orientation;\n"+
				"angles = quat2eul([quat.W quat.X quat.Y quat.Z]);"+
				"oz = angles(:,1);\n"+
				"odometry = [ox,oy,oz];\n"+
				"odomLine = [double(ot),ox,oy,oz];\n"+
				"for k = 1:size(data,1)\n" +
					"	laserAll(k,:) = [double(ol) data(k,:) scan.Ranges' angle'];\n" +
				"end\n" +	
				"csvwrite('/Users/Timshel/Documents/workspace/"+
				"SparkEKFSlam/data/laserRanges.txt',laserRanges);"+
				"csvwrite('/Users/Timshel/Documents/workspace/"+
				"SparkEKFSlam/data/odometry.txt',odometry);"+
				"csvwrite('/Users/Timshel/Documents/workspace/"+
				"SparkEKFSlam/data/laserData.txt',data);";

	
	private static final String slam2 = 
			"% Compute distance of the closest obstacle\n"+
		      "dist = sqrt(x.^2 + y.^2);\n"+
		      "minDist = min(dist);\n"+
		      "% Command robot action\n"+
		      "if minDist < distanceThreshold\n"+
		          "% If close to obstacle, back up slightly and spin\n"+
		          "velmsg.Angular.Z = spinVelocity;\n"+
		          "velmsg.Linear.X = backwardVelocity;\n"+
		      "else\n"+
		          "% Continue on forward path\n"+
		          "velmsg.Linear.X = forwardVelocity;\n"+
		          "velmsg.Angular.Z = 0;\n"+
		      "end\n"+
		      "send(robot,velmsg);\n";
	
	private static final String  map = 
			"filePath = "+
			"fullfile(fileparts(which('MapUpdateUsingSensorDataExample')), 'data',"+
			"'rosbagWithSensorData.bag');\n"+
			"bag = rosbag(filePath);\n"+
			"[poses,laserRanges] = exampleHelperReadPoseAndSensorMsgs(bag);\n"+
			"map = robotics.BinaryOccupancyGrid(16,16,20)\n"+
			"map.GridLocationInWorld = [-8 -8] \n"+
			"show(map)\n"+
			"for j=1:length(poses)\n"+
				"	q = [poses{j}.Orientation.W poses{j}.Orientation.X "+
						"poses{j}.Orientation.Y poses{j}.Orientation.Z];\n"+
				"	orientation = quat2eul(q, 'ZYX');"+
				"	rotMatrixAlongZ = axang2rotm([0 0 1 orientation(1)]);\n"+
				"	rangesInWorldFrame = rotMatrixAlongZ(1:2,1:2) * laserRanges{j}';\n"+
				"	setOccupancy(map, rangesInWorldFrame', 1);\n"+
			"end\n"+
			"show(map)\n"+
			"save('maper', 'map');\n";
			

	//proxy.eval(slam0);


	//plot 
	//proxy.eval("figure");
	
	public ProducerSimConnect(){
		try {
		//start proxy 
		proxy = factory.getProxy();
		//init gazebo
		proxy.eval("ipaddress = '192.168.56.101'");
		proxy.eval("rosinit(ipaddress)");
		proxy.eval("rostopic list");
		//subscribe to scanner
		proxy.eval("robot = rospublisher('/mobile_base/commands/velocity');");
		proxy.eval("velmsg = rosmessage(robot);");
		proxy.eval("laser = rossubscriber('/scan');");
		proxy.eval("odom = rossubscriber('/odom')");
		//proxy.eval("rostopic echo odom;");
		//set physics params
		proxy.eval("spinVelocity = 0.6;"); // Angular velocity (rad/s)  
		proxy.eval("forwardVelocity = 0.1;"); //Linear velocity (m/s)
		proxy.eval("backwardVelocity = -0.02;");//  Linear velocity (reverse) (m/s)
		proxy.eval("distanceThreshold = 0.6;"); // Distance threshold (m) for turning");
		proxy.eval("odometry= [];");
		/*
			proxy.eval("t = tcpip('localhost',10555);");
			proxy.eval("t.OutputBufferSize = 7000;");
			proxy.eval("t.timeout = 30;");
			proxy.eval("fopen(t);");
			//obstacle avoidance
			//proxy.eval("scan = receive(laser);\n data = readCartesian(scan);");
		
		 */
		
		}catch (Exception e){
			e.printStackTrace();
		}
		
	}
	
	public void send(String s){
		try {
			proxy.eval(s);
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public double[] getOdom(){
		try {
			Object[] data = proxy.returningEval("odomLine", 1);
			double[] tmp = (double[]) data[0];
			//System.out.println(tmp[0] + " " + tmp[1] + " " + tmp[2]);
			//System.out.println(tmp.length);
			return tmp;
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	public float[][] getLaser(){
		try {
			//data is 616 X 643 cols
			//totally 396088, appended by column
			//0-615 time stamps
			//616-1231 x
			//1231-1847 y
			//1847 + 640= 2488 ranges
			Object[] data = proxy.returningEval("laserAll", 1);
			float[] tmp = (float[]) data[0];
			//convert to array[][]
			int n = 0;
			float[][] f = new float[616][1283];
			for(int j = 0; j < 643 && n < tmp.length; j++){
				for(int i = 0; i < 616; i++){
					//get column
					if(tmp.length > (n + i))
						f[i][j] = tmp[n + i];
					else break;
				} n += 615;
			}
			/*System.out.println(tmp[0] + " " + tmp[615] + " " + tmp[616] +" "+ tmp[1231] + " " + tmp[1847]);
			System.out.println(tmp[tmp.length-1]);
			System.exit(0);
			*/
		
			return f;
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	public void slam1(){
		try {
			proxy.eval(slam1);
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void slam2(){
		try {
			proxy.eval(slam2);
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void getMap(){
		try {
			proxy.eval(map);
		} catch (MatlabInvocationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void disconnect(){
		proxy.disconnect();
	}
}
