import java.util.ArrayList;
import java.util.List;

import edu.wpi.cscore.*;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;

import org.opencv.core.*; 
import org.opencv.core.Core.*; 
import org.opencv.features2d.FeatureDetector; 
import org.opencv.imgcodecs.Imgcodecs; 
import org.opencv.imgproc.*; 
import org.opencv.objdetect.*;

public class Main {
	private static Mat hsvThresholdOutput;
	private static Mat cvErodeOutput;
	private static Mat cvDilateOutput;
	private static ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private static ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
		
	public static void main(String[] args) {
	    // Loads our OpenCV library. This MUST be included 
	    System.loadLibrary("opencv_java310");

			// Custom UDP connection to rio
			// \/------\/
			// /\------/\

			// HTTP Camera
			HttpCamera frontCamera = new HttpCamera("FrontCam MJPEG", 
				"http://192.168.1.88:1185/stream.mjpg", HttpCameraKind.kMJPGStreamer);
			System.out.println(frontCamera.toString());
			if(!frontCamera.isValid())
			{
				System.out.println("No stream found");
				return;
			}

	    // This creates a CvSink for us to use. This grabs images from our selected camera,
	    // and will allow us to use those images in opencv
			CvSink imageSink = new CvSink("CV Image Grabber");
	    imageSink.setSource(frontCamera);

	    // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
			// operation
			CvSource processedImageSource = new CvSource("CV Image Source", 
				VideoMode.PixelFormat.kMJPEG, 1280, 720, 60);
	    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1187);
			cvStream.setSource(processedImageSource);
			System.out.println(">> MJPEG for processed image created on port 1187");

	    // All Mats and Lists should be stored outside the loop to avoid allocations
			// as they are expensive to create
	    Mat inputImage = new Mat();
			Mat processedImage = new Mat();
			hsvThresholdOutput = new Mat();
			cvErodeOutput = new Mat();
			cvDilateOutput = new Mat();

			// Infinitely process image
			System.out.println("Entering processing loop...");
	    while (true) {
				long frameTime = imageSink.grabFrame(inputImage);
				if (frameTime == 0) 
				{
					System.out.println(imageSink.getError());
					continue;
				}				

	      // REST OF THE VISION CODE HERE
				// \/--------\/
				System.out.println("Processing Image");
				long startTime = System.nanoTime();
				processedImage = ProcessImage(inputImage);
				System.out.println("Process time: " + (System.nanoTime() - startTime) / 1000000 + "ms");
	      
	      if(filterContoursOutput.isEmpty())
	      {
	    	  System.out.println("No targets found");
	    	  //nt_vision.getEntry("targetFound").setBoolean(false);
	    	  // Set Arduino lights to red
	      }
	      else
	      {
	    		// put rectangle(s) over the picture based on filtered contours
		      int x = 0;
		      for (MatOfPoint mop : filterContoursOutput) 
		      {
		    	  // calculate angle from center and estimated distance
		    	  Rect rect = Imgproc.boundingRect(mop);
		    	  double targetMidX = rect.x + (rect.width / 2);
		    	  double angleFromCenter = GetAngleFromCenter(78, targetMidX, inputImage.width());
		    	  double estimatedDistance = GetEstimatedDistance(78, mop.height(), inputImage.height());
		    	  
		    	  Imgproc.rectangle(processedImage, new Point(rect.x, rect.y), 
		    			  new Point(rect.x + rect.width, rect.y + rect.height), 
		    			  new Scalar(255, 0, 0, 255), 3);
		    	  Imgproc.putText(processedImage,  "AFC: " + angleFromCenter, 
		    			  new Point(rect.x + rect.width + 3, rect.y + 5), 0, 12.5, 
		    			  new Scalar(255, 0, 0, 255));
		    	  Imgproc.putText(processedImage,  "ED: " + estimatedDistance, 
		    			  new Point(rect.x + rect.width + 3, rect.y + 18), 0, 12.5, 
		    			  new Scalar(255, 0, 0, 255));
		    	  
		    	  // nt_vision.getEntry("targetFound").setBoolean(true);
		    	  // nt_vision.getEntry("target" + x).setDoubleArray(new double[] {
		    		// 	  angleFromCenter,
		    		// 	  estimatedDistance
		    	  // });
		    	  
		    	  x++;
		      }
		      // Set arduino lights to green
	      }
	      // /\--------/\

	      // Here is where you would write a processed image that you want to restreams
	      // This will most likely be a marked up image of what the camera sees
	      // For now, we are just going to stream the HSV image
				processedImageSource.putFrame(inputImage);
			}			
		}

	  private static Mat ProcessImage(Mat source0) {
	    // Step HSV_Threshold0:
			Mat hsvThresholdInput = source0;
			double[] hsvThresholdHue = {69.60431654676259, 120.9090909090909};
			double[] hsvThresholdSaturation = {190.33273381294964, 255.0};
			double[] hsvThresholdValue = {77.96762589928058, 255.0};
			hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

			// Step CV_erode0:
			Mat cvErodeSrc = hsvThresholdOutput;
			Mat cvErodeKernel = new Mat();
			Point cvErodeAnchor = new Point(-1, -1);
			double cvErodeIterations = 1.0;
			int cvErodeBordertype = Core.BORDER_CONSTANT;
			Scalar cvErodeBordervalue = new Scalar(-1);
			cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

			// Step CV_dilate0:
			Mat cvDilateSrc = cvErodeOutput;
			Mat cvDilateKernel = new Mat();
			Point cvDilateAnchor = new Point(-1, -1);
			double cvDilateIterations = 3.0;
			int cvDilateBordertype = Core.BORDER_CONSTANT;
			Scalar cvDilateBordervalue = new Scalar(-1);
	    cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

			// Step Find_Contours0:
			Mat findContoursInput = cvDilateOutput;
			boolean findContoursExternalOnly = false;
			findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

			// Step Filter_Contours0:
			ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
			double filterContoursMinArea = 0.0;
			double filterContoursMinPerimeter = 0.0;
			double filterContoursMinWidth = 0.0;
			double filterContoursMaxWidth = 1000.0;
			double filterContoursMinHeight = 0.0;
			double filterContoursMaxHeight = 1000.0;
			double[] filterContoursSolidity = {0, 100};
			double filterContoursMaxVertices = 1000000.0;
			double filterContoursMinVertices = 0.0;
			double filterContoursMinRatio = 0.0;
			double filterContoursMaxRatio = 1000.0;
	    filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
	    
	    return cvDilateOutput;
	  }
	  
	  /**
	    * Segment an image based on hue, saturation, and value ranges.
	    *
	    * @param input The image on which to perform the HSL threshold.
	    * @param hue The min and max hue
	    * @param sat The min and max saturation
	    * @param val The min and max value
	    * @param output The image in which to store the output.
	    */
	  private static void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
	    Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
	    Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
	      new Scalar(hue[1], sat[1], val[1]), out);
	  }

	  /**
	    * Expands area of lower value in an image.
	    * @param src the Image to erode.
	    * @param kernel the kernel for erosion.
	    * @param anchor the center of the kernel.
	    * @param iterations the number of times to perform the erosion.
	    * @param borderType pixel extrapolation method.
	    * @param borderValue value to be used for a constant border.
	    * @param dst Output Image.
	    */
	  private static void cvErode(Mat src, Mat kernel, Point anchor, double iterations, int borderType, Scalar borderValue, Mat dst) {
	    if (kernel == null) {
	      kernel = new Mat();
	    }
	    if (anchor == null) {
	      anchor = new Point(-1,-1);
	    }
	    if (borderValue == null) {
	      borderValue = new Scalar(-1);
	    }
	    Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	  }

	  /**
	    * Expands area of higher value in an image.
	    * @param src the Image to dilate.
	    * @param kernel the kernel for dilation.
	    * @param anchor the center of the kernel.
	    * @param iterations the number of times to perform the dilation.
	    * @param borderType pixel extrapolation method.
	    * @param borderValue value to be used for a constant border.
	    * @param dst Output Image.
	    */
	  private static void cvDilate(Mat src, Mat kernel, Point anchor, double iterations, int borderType, Scalar borderValue, Mat dst) {
	    if (kernel == null) {
	      kernel = new Mat();
	    }
	    if (anchor == null) {
	      anchor = new Point(-1,-1);
	    }
	    if (borderValue == null){
	      borderValue = new Scalar(-1);
	    }
	    Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	  }

	  /**
	    * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	    * @param input The image on which to perform the Distance Transform.
	    * @param type The Transform.
	    * @param maskSize the size of the mask.
	    * @param output The image in which to store the output.
	    */
	  private static void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
	    Mat hierarchy = new Mat();
	    contours.clear();
	    int mode;
	    if (externalOnly) {
	      mode = Imgproc.RETR_EXTERNAL;
	    }
	    else {
	      mode = Imgproc.RETR_LIST;
	    }
	    int method = Imgproc.CHAIN_APPROX_SIMPLE;
	    Imgproc.findContours(input, contours, hierarchy, mode, method);
	  }

	  /**
	    * Filters out contours that do not meet certain criteria.
	    * @param inputContours is the input list of contours
	    * @param output is the the output list of contours
	    * @param minArea is the minimum area of a contour that will be kept
	    * @param minPerimeter is the minimum perimeter of a contour that will be kept
	    * @param minWidth minimum width of a contour
	    * @param maxWidth maximum width
	    * @param minHeight minimum height
	    * @param maxHeight maximimum height
	    * @param Solidity the minimum and maximum solidity of a contour
	    * @param minVertexCount minimum vertex Count of the contours
	    * @param maxVertexCount maximum vertex Count
	    * @param minRatio minimum ratio of width to height
	    * @param maxRatio maximum ratio of width to height
	    */
	  private static void filterContours(List<MatOfPoint> inputContours, double minArea,
	    double minPerimeter, double minWidth, double maxWidth, double minHeight, double
	    maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
	    minRatio, double maxRatio, List<MatOfPoint> output) {
	    final MatOfInt hull = new MatOfInt();
	    output.clear();
	    //operation
	    for (int i = 0; i < inputContours.size(); i++) {
	      final MatOfPoint contour = inputContours.get(i);
	      final Rect bb = Imgproc.boundingRect(contour);
	      if (bb.width < minWidth || bb.width > maxWidth) continue;
	      if (bb.height < minHeight || bb.height > maxHeight) continue;
	      final double area = Imgproc.contourArea(contour);
	      if (area < minArea) continue;
	      if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
	      Imgproc.convexHull(contour, hull);
	      MatOfPoint mopHull = new MatOfPoint();
	      mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
	      for (int j = 0; j < hull.size().height; j++) {
	        int index = (int)hull.get(j, 0)[0];
	        double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
	        mopHull.put(j, 0, point);
	      }
	      final double solid = 100 * area / Imgproc.contourArea(mopHull);
	      if (solid < solidity[0] || solid > solidity[1]) continue;
	      if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
	      final double ratio = bb.width / (double)bb.height;
	      if (ratio < minRatio || ratio > maxRatio) continue;
	      output.add(contour);
	    }
	  }
	  
	  
	  
	  private static double GetAngleFromCenter(int horizontalFov, double targetMidX, int frameWidth) 
	  {
		  // This method is for horizontal calculations
		  double frameMidX = frameWidth / 2;
		  double anglePerPixel = horizontalFov / frameWidth;
		  
		  // First, get difference from targetMidX to middle of the frame in pixels
		  // Last, find estimated angle difference from center
		  double angleFromCenter = (frameMidX - targetMidX) * anglePerPixel;
		  
		  return angleFromCenter;
	  }
	  
	  
	  private static double GetVerticalAngle(int verticalFov, int heightOfTarget, int frameHeight)
	  {
		  double anglePerPixel = verticalFov / frameHeight;
		  double totalAngleOfTarget = heightOfTarget * anglePerPixel;
		  return totalAngleOfTarget;
	  }
	  
	  private static double GetEstimatedDistance(int verticalFov, int targetHeight, int frameHeight)
	  {
		  // Main equation: 
		  // d2t = (H/2) / tan(angle/2)
		  double angleSegmentOfTarget = GetVerticalAngle(verticalFov, targetHeight, frameHeight); 
		  
		  double estimatedDistance = (targetHeight / 2) / Math.tan(angleSegmentOfTarget / 2);
		  return estimatedDistance;
	  }
}
