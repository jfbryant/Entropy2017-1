package org.usfirst.frc.team138.robot.subsystems.vision2017;

import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team138.robot.Sensors;
import java.io.File;
import java.io.FileNotFoundException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class Entropy2017Targeting extends Thread {
	private static final double inchesFromCenter = 5;
	private static final double xSpaceInchesConst = 6.25;
	private static final double angleConstant = 17.0;
	
	private static final boolean DEBUG_OUTPUT_ENABLED = false;
	
	// Widget Name
	public static final String NAME = "ENTROPY-2017-TARGETING 2:18:2017";
	
	// Random unique identifier for the camera - leave this - otherwise, you will get a warning
	private static final long serialVersionUID = -412351776843653585L;
	
	// Paths
	static String rootFolder = "C:\\Users\\Team138\\Vision2017";
	private String  SmartDashboardPath = "/SmartDashboard/extensions/";
	
	enum Parameters	{
		lowHue,
		highHue,
		lowSat,
		highSat,
		highVal,
		lowVal,
		DilateSize,   // 4.0
		DilateKernel,
		CROSS_HAIR_X,
		CROSS_HAIR_Y,
		CROSS_HAIR_SIZE,
		MilliSecsBetweenPics,   // 1000
		ErodeSize,   // 4.0
		AspectMin,   // 1.0
		AspectMax,   // 5.0
		AreaMin,     // 3000
		AreaMax,     // 5000
		HeightMin,   // 25.0
		SaveRaw,
		SaveProcessed
	};
	
	public class TargetInformation{
		public String targetType = "peg"; //"peg" or "highGoal"
		public boolean targetFound = false;
		public long highGoalx = 0;
		public long highGoaly = 0;
		public long highGoalxRange = 0;
		public long pegyHeight = 0;
		public long pegxSpace = 0;
		public long pegx = 0;
		public double realPegX = 0;
		public long pegy = 0;
		public double correctionAngle = 0;
	}
	
	// Keep track of frame number and processing results
	private long m_frameNumber = 0;
	private ArrayList<TargetInformation> infoList = new ArrayList<TargetInformation>();
	
	// Date object for saving images
	private Date m_lastSnapshotTime;
	private Date m_lastRawSnapshotTime;

	private Properties theProperties;

	private int framesToProcess = 0;
	private String targetProcessingType = "peg";
	private boolean isCancelled = false;
	private boolean done = false;
	
	public Entropy2017Targeting() {
		m_frameNumber = 0;
		m_lastSnapshotTime = new Date();
		m_lastRawSnapshotTime = new Date();
		
		loadParameters();
	}
	
	public void run() {        
        CvSink cvSink = CameraServer.getInstance().getVideo();        
        Mat source = new Mat();
        
        while(!done) {
        	synchronized(this)
        	{
        		try {
					this.wait();
				} catch (InterruptedException e) {}
        	}
        	
        	while (framesToProcess > 0)
        	{
        		if (!isCancelled)
        		{
        			cvSink.grabFrame(source);
                    processImage(source);
                    framesToProcess--;
                    System.out.println("Frames to Process: " + framesToProcess);
        		}
        		else
        		{
        			framesToProcess = 0;
        			getTargetInformation();
        			isCancelled = false;
        		}
        	}
        	Sensors.standardCameraMode();
        }
	}
	
	public void shutdownThread()
	{
		done = true;
		this.notify();
	}
	
	public ArrayList<TargetInformation> getTargetInformation()
	{
		@SuppressWarnings("unchecked")
		ArrayList<TargetInformation> temp = (ArrayList<TargetInformation>) infoList.clone();
		infoList.clear();
		return temp;
	}
	
	public void processFrames(int numFrames, String targetType)
	{
		Sensors.targetingCameraMode();
		framesToProcess = numFrames;
		targetProcessingType = targetType;
		synchronized(this)
		{
			this.notify();
		}
	}
	
	public void cancelProcessing()
	{
		if (framesToProcess > 0)
		{
			isCancelled = true;
		}
	}
	
	public static void main(String[] args)
	{
//		String inputFolder = rootFolder + "\\LED Peg";
//		
//		String outputFolder = rootFolder + "\\LEDPeg_output";
//				
//		File folder = new File(inputFolder);
//		File[] listOfFiles = folder.listFiles();
//		
//		Entropy2017Targeting imageProcessor = new Entropy2017Targeting();
//		
//		for(File f : listOfFiles)
//		{
//			System.out.println();
//			System.out.println("---------------------------------------------------------------------------------------------");
//			System.out.println();
//			
//			Mat ourImage = Imgcodecs.imread(f.getPath());
//			System.out.println("File: " + f.getPath());
//			imageProcessor.processImage(ourImage);
//			Imgcodecs.imwrite(outputFolder + "\\"+ f.getName()+".png", imageProcessor.outputImage);
//			
//			
//			System.out.println("Output "+ f.getName());
//		}
//		//imageProcessor.disconnect();
//		System.out.println("Done.");
	}

	protected void processImage(Mat m) {
		m_frameNumber++;
		
		if ( theProperties == null ) {
			//m_error = "parameter table missing";
			return; 
		}
		
		// Blank info struct to store target information
		TargetInformation targetInfo = new TargetInformation();
		targetInfo.targetType = targetProcessingType;
	
		Mat step1 = getHSVThreshold(m);
		
		if (targetInfo.targetType == "peg")
		{
			findPeg(step1, targetInfo);
			if (targetInfo.targetFound)
			{
				drawTarget(m, (long)targetInfo.realPegX, targetInfo.pegy, true);
			}
		}
		else if (targetInfo.targetType == "highGoal")
		{
			findHighGoal(step1, targetInfo);
			if (targetInfo.targetFound)
			{
				drawTarget(m, targetInfo.highGoalx, targetInfo.highGoaly, false);
			}
		}
		
		infoList.add(targetInfo);		
		return;
    }

	/**
	 * Process pixels in the correct color range and cleanup the image
	 * @param m Input image
	 * @return Cleaned up image
	 */
	private Mat getHSVThreshold(Mat m) {
		
		// convert BGR values to HSV values
		Mat hsv = new Mat();
		Imgproc.cvtColor(m, hsv, Imgproc.COLOR_BGR2HSV);
		
		
		Mat inRange = new Mat();
		Core.inRange(
				hsv, 
				new Scalar(Double.parseDouble(theProperties.getProperty("lowHue")), 
					Double.parseDouble(theProperties.getProperty("lowSat")),
					Double.parseDouble(theProperties.getProperty("lowVal"))),
				new Scalar(Double.parseDouble(theProperties.getProperty("highHue")),
							Double.parseDouble(theProperties.getProperty("highSat")),
							Double.parseDouble(theProperties.getProperty("highVal"))), 
				inRange);
		
		Mat grey = new Mat();
		Imgproc.cvtColor(m, grey, Imgproc.COLOR_BGR2GRAY);
		Core.bitwise_and(grey, inRange, grey);

        Imgcodecs.imwrite(rootFolder + "/1_Post_inRange" + ".png", grey);
		
		return grey;
	}
	
	private void findPeg(Mat m, TargetInformation output){
	    long[] xsums = sums(m,true);
	    long[] ysums = sums(m,false);
	    
	    List<PeakLoc> ypeaks = findPeaks(ysums);
	    List<PeakLoc> xpeaks = findPeaks(xsums);
		 
		//locate peaks (all of them) in sums, for peg it will be 2 x peaks, 1 y peak
	    if ((xpeaks.size() == 2) && (ypeaks.size() > 0)){
	    	output.targetFound = true;
	    	output.pegx = (xpeaks.get(1).getStart() + xpeaks.get(0).getStop()) /2;
	    	output.pegxSpace = xpeaks.get(1).getStart() - xpeaks.get(0).getStop();
	    	output.pegyHeight = ypeaks.get(0).getStop() - ypeaks.get(0).getStart();
	    	output.pegy = ypeaks.get(0).getStart() + output.pegyHeight/2;
	    	output.realPegX = output.pegx - output.pegxSpace * inchesFromCenter / xSpaceInchesConst;
	    	output.correctionAngle = (double)((output.realPegX - m.cols() / 2)) / angleConstant;
	    	if (DEBUG_OUTPUT_ENABLED)
	    	{
	    		System.out.println("pegx = " + output.pegx + " , " + "pegxspace = " + 
	    			output.pegxSpace + " , " + "pegyheight = " + output.pegyHeight); 
	    	}
	    }
	    else
	    {
	    	output.targetFound = false;
	    }
	    return;
	}
	
	/**
	 * locate the high goal
	 * @param m
	 */
	private void findHighGoal(Mat m, TargetInformation output) {
		/*
		 * private long highGoalx = 1000;
	       private long highGoaly = 1000;
	       private long highGoalxRange = 1000;
		 */
		
		// crop the image looking at top only
		int heightThreshold = m.height();
		Rect rectCrop = new Rect(0,0,m.width(),heightThreshold);
        Mat imageROI = m.submat(rectCrop);
        
        Imgcodecs.imwrite(rootFolder + "/1_Crop" + ".png", imageROI);
        
        
        // locate all the possible candidates
        long[] xsums = sums(imageROI,true);
        
	    List<PeakLoc> xpeaks = findPeaks(xsums,10);
	    
	    // look for 2 peaks in each candidate
	    for (int k=0;k<xpeaks.size();k++) {
	    	long xstart = xpeaks.get(k).getStart();
	    	long xstop = xpeaks.get(k).getStop();
	    			
	    	// Create an image within the peak
	    	Rect rectQual = new Rect((int)xstart,0,(int)(xstop-xstart),heightThreshold);
	        Mat imageQual = m.submat(rectQual);
	        
	        // sum within y within the peak
	        long[] ysums = sums(imageQual,false);
		    List<PeakLoc> ypeaks = findPeaks(ysums,10);
		    
		    // only process if we have 2 peaks (two circles)
		    if (ypeaks.size() == 2) {
		    	int height0 = (int) (ypeaks.get(0).getStop() - ypeaks.get(0).getStart());
		    	int height1 = (int) (ypeaks.get(1).getStop() - ypeaks.get(1).getStart());
		    	int space = (int) (ypeaks.get(0).getStop() - ypeaks.get(1).getStart());
		    	
		    	// compute the ratios; top is twice as big as bottom and space is same as bottom
		    	double heightRatio = (double)height0/height1;
		    	double spaceRatio = (double)space/height1;
		    	
		    	// check the porportions
		    	double hightRatioTol = 0.25;
		    	double spacetRatioTol = 0.5;
		    	if ((Math.abs(heightRatio-2.0) <= hightRatioTol) && ((Math.abs(spaceRatio-1.0) <= spacetRatioTol))) {
		    		// save targeting info and quit
		    		output.highGoalx = (xstop+xstart)/2;
		    		output.highGoaly = ypeaks.get(0).getStop();
		    		output.highGoalxRange = xstop-xstart;
		    		break;
		    	} // size matches
		    } // #peaks in y is 2
	    } // for each peak in x
	}
	
	/**
	 * 
	 * @param m
	 * @param x
	 * @param y
	 */
	private void drawTarget(Mat m, long x, long y, boolean peg){
		
		int size = 40;
		Scalar color;
		if (peg) {
			color = new Scalar(0,0,255);
		}
		else {
			color = new Scalar(0,255,255);
		}
		
		Imgproc.line(m, new Point(x-size,y), new Point(x+size,y), color,2);
		Imgproc.line(m, new Point(x,y-size), new Point(x,y+size), color,2);
		for (int k =0;k<4;k++) {
			Imgproc.circle(m, new Point(x,y), (k+1)*size/4, color);
		}
		
	}
	
	/**
	 * Sum the rows or columns of a matrix
	 * @param m input 2D matrix as unsigned bytes
	 * @param byRow true is to sumy the matrix by row; otherwise by colums
	 * @return integer array of sums
	 */
	private static long[] sums(Mat m,boolean byRow) {
		
		int rows = m.rows();
		int cols = m.cols();
		byte[] data = new byte[rows*cols];
		long[] retSums = null;
		
		int status = m.get(0, 0,data);
		
		long total = 0;
		for (int k=0;k<data.length;k++) {
			total += Byte.toUnsignedInt(data[k]);
		}
		
		if (byRow) {
			retSums = new long[cols];
			for (int col=0;col<cols;col++) {
				retSums[col] = 0;
				for (int row=0;row<rows;row++) {
					int k = row*cols+col;
					retSums[col] += Byte.toUnsignedInt(data[k]);
				}
			}
		}
		else {
			retSums = new long[rows];
			for (int row=0;row<rows;row++) {
				retSums[row] = 0;
				for (int col=0;col<cols;col++) {
					int k = row*cols+col;
					retSums[row] += Byte.toUnsignedInt(data[k]);
				}
			}
		}
		
		int total1 = 0;
		for (int k=0; k < retSums.length; k++) {
			total1 += retSums[k];
		}
		
		if (DEBUG_OUTPUT_ENABLED)
		{
			System.out.println("" + m.rows()+ "  " + m.cols());
			System.out.println(status);
			System.out.println(total);
			System.out.println(total1);
		}
	
		return retSums;
	}
	
	/**
	 * Debug routine to print peaks
	 * @param name
	 * @param peaks
	 */
	private static void printPeaks(String name,List<PeakLoc> peaks) {
		System.out.println("Peaks for " + name);
		for (int k=0;k<peaks.size();k++) {
			System.out.println("peak[" + k + "]= " + peaks.get(k).getStart() + " , " + peaks.get(k).getStop());
		}
	}
	
	/**
	 * Peak location
	 * @author Team138
	 *
	 */
	private static class PeakLoc {
		private long start;
		private long stop;
		public PeakLoc(long start, long stop) {
			super();
			this.start = start;
			this.stop = stop;
		}
		public long getStart() {
			return start;
		}
		public void setStart(long start) {
			this.start = start;
		}
		public long getStop() {
			return stop;
		}
		public void setStop(long stop) {
			this.stop = stop;
		}
	}
	
	public static void main1(String[] args) {
		
		System.out.println("Start");
		long[] testData = new long[10];
		for (int k=0;k<testData.length;k++) {
			testData[k] = 0;
		}
		testData[2] = 10;
		testData[3] = 12;
		testData[7] = 15;
		testData[8] = 19;
		
		List<PeakLoc> peaks = findPeaks(testData);
		System.out.println(peaks);
	}
	
	private static List<PeakLoc> findPeaks(long[] sums,long minWidth) {
		long maxVal = Arrays.stream(sums).max().getAsLong();
		ArrayList<PeakLoc> ret = new ArrayList<PeakLoc>();
		boolean looking = true;
		long start = 0;
		
		for (int k=0;k<sums.length;k++) {
			if (looking){
				if ((sums[k]) > (maxVal/2)){
					looking = false;
					start = k;
				}
			}
			else{
				if ((sums[k]) < (maxVal/4)){
					looking = true;
					long width = (k-1)-start;
					if (width >= minWidth) {
					    ret.add(new PeakLoc(start, k-1));
					}
				}
			}
			
		}
		if (looking == false){
			ret.add(new PeakLoc(start, sums.length - 1));
		}
		
		return ret;
	}
	
	private static List<PeakLoc> findPeaks(long[] sums) {
		return findPeaks(sums, 3);
	}

	private void saveProcessedImage(Mat m) {
		//if (m_pt.get(Parameters.SaveProcessed.ordinal()) > 0.5) {
		if (Double.parseDouble(theProperties.getProperty("SaveProcessed", "0.0")) > 0.5) {
			Date now = new Date();
			if (now.getTime() - m_lastSnapshotTime.getTime() >= Double.parseDouble(theProperties.getProperty("MilliSecsBetweenPics"))) {
				String fileName = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss_SSS").format(now);
				Imgcodecs.imwrite("C:/StrongholdImages/Camera_" + fileName + ".jpg", m);
				m_lastSnapshotTime = new Date();
			}
		}
	}
	
	private void saveRawImage(Mat m) {
		if (Double.parseDouble(theProperties.getProperty("SaveRaw", "0.0")) > 0.5) {
			Date now = new Date();
			if (now.getTime() - m_lastRawSnapshotTime.getTime() >= 1000) {
				String fileName = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss_SSS").format(now);
				Imgcodecs.imwrite("C:/StrongholdRawImages/Camera_" + fileName + ".jpg", m);
				m_lastRawSnapshotTime = new Date();
			}
		}
	}
	
	private void loadParameters() {
		theProperties = new Properties();
		
		theProperties.setProperty("lowHue", "70");
		theProperties.setProperty("highHue", "100");
		theProperties.setProperty("lowSat", "100");
		theProperties.setProperty("highSat", "255");
		theProperties.setProperty("lowVal", "50");
		theProperties.setProperty("highVal", "255");
		theProperties.setProperty("DilateSize", "4.0");
		theProperties.setProperty("DilateKernel", "2");
		theProperties.setProperty("CROSS_HAIR_X", "200");
		theProperties.setProperty("CROSS_HAIR_Y", "100");
		theProperties.setProperty("CROSS_HAIR_SIZE", "30");
		theProperties.setProperty("MilliSecsBetweenPics", "1000");
		theProperties.setProperty("ErodeSize", "4.0");
		theProperties.setProperty("AspectMin", "0.5");
		theProperties.setProperty("AspectMax", "5.0");
		theProperties.setProperty("AreaMin", "1000");
		theProperties.setProperty("AreaMax", "5000");
	}

}
