//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Media3D;
    using System.Runtime.InteropServices;
    using System.Windows.Interop;
    using System.Threading.Tasks;
    using Microsoft.Kinect;

   // using Lifx.Lib;
    using System.Net;
    using System.Net.Sockets;
    using System.Text;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        public List<Grip> grips = new List<Grip>();

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private static Recorder recorder = new Recorder();

        private static List<Hotspot> hotspots = new List<Hotspot>();

        private static String mode = "run";

        private static Interface colorBlock = new Interface("color", 1);
        private static Interface sysAud = new Interface("sound", 1);
        private static Interface recordMode = new Interface("record", 1);


        private Stopwatch leftWatch = new Stopwatch();
        private Stopwatch rightWatch = new Stopwatch();

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            setupHotspots();


        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    dc.DrawRectangle(colorBlock.brush, null, new Rect(this.displayWidth - 150, this.displayHeight - 150, 150, 150));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                            //IReadOnlyDictionary<JointType, JointOrientation> orientations = body.JointOrientations;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);

                            // MY BIT
                            if (mode == "run")
                            {
                                manageGrips(body, joints);

                                if (hotspots != null)
                                {
                                    foreach (Hotspot hotspot in hotspots)
                                    {
                                        checkForNewGrips(body, joints, hotspot, ref grips);
                                    }
                                }
                            }
                            else if (mode == "record")
                            {
                                // when in record mode go through all the hotspots one at a time
                                // display the type and the id and the vector to be recorded (1 of 2, or 2 of 2)
                                // user records each vector by pointing at object and clenching fist 
                                // once this has been done twice from two different locations we find point of intersection of the vectors
                                // or the nearest point on one line and save this as the coords of the hotspot
                                // finally we reinit all hotspots and move to run mode

                                if (body.HandLeftState == HandState.Closed)
                                {
                                    if (!leftWatch.IsRunning)
                                    {
                                        leftWatch.Start();
                                    }
                                    else if (leftWatch.ElapsedMilliseconds > 1500)
                                    {
                                        leftWatch.Reset();
                                        recorder.addPoint(joints[JointType.ElbowLeft].Position, joints[JointType.WristLeft].Position);
                                    }
                                }
                                else
                                {
                                    leftWatch.Reset();
                                }

                                if (body.HandRightState == HandState.Closed)
                                {
                                    if (!rightWatch.IsRunning)
                                    {
                                        rightWatch.Start();
                                    }
                                    else if (rightWatch.ElapsedMilliseconds > 1500)
                                    {
                                        rightWatch.Reset();
                                        recorder.addPoint(joints[JointType.ElbowRight].Position, joints[JointType.WristRight].Position);
                                    }
                                }
                                else
                                {
                                    rightWatch.Reset();
                                }

                            }
                        }

                        // prevent drawing outside of our render area
                        this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                }
            }
        }

        public class Recorder
        {
            List<Vector3D> elbows = new List<Vector3D>();
            List<Vector3D> wrists = new List<Vector3D>();

            public void record()
            {
                mode = "record";
                Console.Beep();
            }

            public void addPoint(CameraSpacePoint elbow, CameraSpacePoint wrist)
            {

                elbows.Add(jointToVector(elbow));
                wrists.Add(jointToVector(wrist));

                Console.WriteLine("Added point " + elbows.Count);
                Console.Beep();

                if (elbows.Count == 2)
                {
                    savePoint();
                }
            }

            private void savePoint()
            {
                Vector3D v1 = Vector3D.Subtract(wrists[0], elbows[0]);
                Vector3D v2 = Vector3D.Subtract(wrists[1], elbows[1]);
                Vector3D intersectionPoint = intersectionOfLines(v1, v2, elbows[0], elbows[1]);

                Console.WriteLine("New intersection point is: " + intersectionPoint.ToString());
                Properties.Settings.Default.hot1Vec = intersectionPoint;
                done();
            }

            private void done()
            {
                mode = "run";
                elbows.Clear();
                wrists.Clear();
                setupHotspots();
            }


        }

        private static void setupHotspots()
        {
            // color
            // hotspots.Add(new Hotspot(Properties.Settings.Default.hot1Vec, colorBlock));

            // volume
            // hotspots.Add(new Hotspot(Properties.Settings.Default.hot2Vec, sysAud));
            hotspots.Clear();

            hotspots.Add(new Hotspot(Properties.Settings.Default.hot1Vec, colorBlock));

            // volume
            hotspots.Add(new Hotspot(Properties.Settings.Default.hot2Vec, sysAud));

            hotspots.Add(new Hotspot(new Vector3D(0, 0, 0), recordMode));
        }


        private static bool isPointingToSky(IReadOnlyDictionary<JointType, Joint> joints)
        {
            bool pointingToSky = false;

            Vector3D up = new Vector3D(0, 1, 0);

            Vector3D lArmVector;
            Vector3D lElbowCoords;
            getLeftArmVector(joints, out lElbowCoords, out lArmVector);

            Vector3D rArmVector;
            Vector3D rElbowCoords;
            getRightArmVector(joints, out rElbowCoords, out rArmVector);

            lArmVector.Normalize();
            rArmVector.Normalize();

            double lUp = Vector3D.DotProduct(lArmVector, up);
            double rUp = Vector3D.DotProduct(rArmVector, up);

            if (lUp > 0.9 || rUp > 0.9)
            {
                pointingToSky = true;
            }

            return pointingToSky;
        }

        private void manageGrips(Body body, IReadOnlyDictionary<JointType, Joint> joints)
        {
            // check all grips, remove any that are no longer being held
            // interact with the ones that are still being held
            if (grips != null)
            {
                List<Grip> removeList = new List<Grip>();
                foreach (Grip grip in grips)
                {
                    if (grip.trackingId == body.TrackingId)
                    {
                        // remove if no longer active
                        HandState handState;
                        Vector3D handPosition;
                        if (grip.hand == 1)
                        {
                            handState = body.HandLeftState;
                            handPosition = getLeftHandCoordinates(joints);
                        }
                        else
                        {
                            handState = body.HandRightState;
                            handPosition = getRightHandCoordinates(joints);
                        }
                        grip.prevCoords = handPosition;


                        if (handState != HandState.Closed && handState != HandState.Unknown)
                        {
                            removeList.Add(grip);
                            grip.hotspot.iface.drop();
                        }
                        else
                        {
                            grip.prevCoords = handPosition;
                            // work out movement in 2d (for horizontal and vertical)
                            Vector3D hVect = getHorizontalDisplacementVect(grip, joints);
                            int sign;
                            if (Math.Abs(hVect.X) > Math.Abs(hVect.Z))
                            {
                                sign = Math.Sign(hVect.X);
                            }
                            else
                            {
                                sign = Math.Sign(hVect.Z);
                            }
                            if (sign == 0)
                            {
                                sign = 1;
                            }

                            double hDisp = hVect.Length * sign;

                            // translate horizontal and vertical movement into output
                            Vector3D yVect = Vector3D.Subtract(handPosition, grip.startCoords);

                            //Console.WriteLine("Horizontal Disp: " + hDisp.ToString() + " Vert: " + yVect.Y);
                            grip.hotspot.iface.onChange(hDisp, yVect.Y);
                        }
                    }


                }

                if (removeList != null)
                {
                    grips.RemoveAll(x => removeList.Contains(x));
                }

                /*foreach (Grip grip in grips)
                {
                    //Console.WriteLine("Grip, hand: " + grip.hand.ToString() + " coords: " + grip.startCoords.ToString());
                }*/

            }
        }

        private static Vector3D getRightHandCoordinates(IReadOnlyDictionary<JointType, Joint> joints)
        {
            Vector3D handPosition;
            handPosition = new Vector3D(joints[JointType.HandRight].Position.X, joints[JointType.HandRight].Position.Y, joints[JointType.HandRight].Position.Z);
            return handPosition;
        }

        private static Vector3D getLeftHandCoordinates(IReadOnlyDictionary<JointType, Joint> joints)
        {
            Vector3D handPosition;
            handPosition = new Vector3D(joints[JointType.HandLeft].Position.X, joints[JointType.HandLeft].Position.Y, joints[JointType.HandLeft].Position.Z);
            return handPosition;
        }

        private static Vector3D jointToVector(CameraSpacePoint joint)
        {
            return new Vector3D(joint.X, joint.Y, joint.Z);
        }

        private static Vector3D intersectionOfLines(Vector3D v1, Vector3D v2, Vector3D o1, Vector3D o2)
        {
            double lambda = (o2.Y - o1.Y + (o1.X - o2.X) * v2.Y / v2.X) / (v1.Y - v1.X * v2.Y / v2.X);
            Vector3D poi = lambda * v1 + o1;
            return poi;
        }

        private Vector3D getHorizontalDisplacementVect(Grip grip, IReadOnlyDictionary<JointType, Joint> joints)
        {
            Vector3D normal = grip.startVect;
            normal.Normalize();

            Vector3D elbowCoords;
            Vector3D armVector;
            Vector3D handCoords;
            if (grip.hand == 1)
            {   // left
                getLeftArmVector(joints, out elbowCoords, out armVector);
                handCoords = getLeftHandCoordinates(joints);
            }
            else
            {   // right
                getRightArmVector(joints, out elbowCoords, out armVector);
                handCoords = getRightHandCoordinates(joints);
            }

            Vector3D proj = Vector3D.Multiply(Vector3D.DotProduct(armVector, normal), normal);
            Vector3D dispVect = Vector3D.Subtract(armVector, proj);
            dispVect.Y = 0;
            return dispVect;
        }

        private void checkForNewGrips(Body body, IReadOnlyDictionary<JointType, Joint> joints, Hotspot hotspot, ref List<Grip> grips)
        {
            double angle = 15.0;
            // loop through each hotspot coord and see if we're poiting at it
            // if so it'll be added to our list of grips (if we're gripping)
            bool pointing = isPointingAt(body, joints, hotspot, angle, ref grips);

        }

        /// <summary>
        /// Ensure that the hand of the person is not already interacting with an object 
        /// </summary>
        private static bool checkGripExistance(Grip grip, ref List<Grip> grips)
        {
            bool gripExists = false;
            if (grips != null)
            {
                foreach (Grip egrip in grips)
                {
                    if (egrip.trackingId == grip.trackingId && egrip.hand == grip.hand)
                    {
                        gripExists = true;
                    }
                }
            }

            return gripExists;
        }

        /// <summary>
        /// Determine if body is pointing at a given hotspot 
        /// </summary>
        /// <param name="joints">Joints of the body</param>
        /// <param name="hotspotCoords">the coordinates of the hostpot to be tested for intersection</param>
        /// <param name="angle">angle (in degreese) for tolerance above or below</param>
        private bool isPointingAt(Body body, IReadOnlyDictionary<JointType, Joint> joints, Hotspot hotspot, double angle, ref List<Grip> grips)
        {
            bool isPointing = false;

            Vector3D lElbowCoords;
            Vector3D lArmVector;
            getLeftArmVector(joints, out lElbowCoords, out lArmVector);

            Vector3D rElbowCoords;
            Vector3D rArmVector;
            getRightArmVector(joints, out rElbowCoords, out rArmVector);

            // left hand

            if (isIntersecting(hotspot.coords, lArmVector, lElbowCoords, angle))
            {
                isPointing = true;
                if (body.HandLeftState == HandState.Closed)
                {
                    Vector3D handPosition = new Vector3D(joints[JointType.HandLeft].Position.X, joints[JointType.HandLeft].Position.Y, joints[JointType.HandLeft].Position.Z);
                    Grip grip = new Grip(body.TrackingId, 1, handPosition, hotspot, lArmVector);
                    // if grip does not already exist
                    if (!checkGripExistance(grip, ref grips))
                    {
                        grips.Add(grip);
                    }
                }
            }

            // right hand
            if (isIntersecting(hotspot.coords, rArmVector, rElbowCoords, angle))
            {
                isPointing = true;
                if (body.HandRightState == HandState.Closed)
                {
                    Vector3D handPosition = new Vector3D(joints[JointType.HandRight].Position.X, joints[JointType.HandRight].Position.Y, joints[JointType.HandRight].Position.Z);
                    Grip grip = new Grip(body.TrackingId, 2, handPosition, hotspot, rArmVector);

                    // if grip does not already exist
                    if (!checkGripExistance(grip, ref grips))
                    {
                        grips.Add(grip);
                        grip.hotspot.iface.pickup(joints);
                    }
                }
            }

            return isPointing;
        }

        private static void getRightArmVector(IReadOnlyDictionary<JointType, Joint> joints, out Vector3D rElbowCoords, out Vector3D rArmVector)
        {
            Vector3D rWristCoords = new Vector3D(joints[JointType.WristRight].Position.X, joints[JointType.WristRight].Position.Y, joints[JointType.WristRight].Position.Z);
            rElbowCoords = new Vector3D(joints[JointType.ElbowRight].Position.X, joints[JointType.ElbowRight].Position.Y, joints[JointType.ElbowRight].Position.Z);
            rArmVector = Vector3D.Subtract(rWristCoords, rElbowCoords);
            //Console.WriteLine("Evg right: " + rWristCoords.ToString());

        }

        private static void getLeftArmVector(IReadOnlyDictionary<JointType, Joint> joints, out Vector3D lElbowCoords, out Vector3D lArmVector)
        {
            Vector3D lWristCoords = new Vector3D(joints[JointType.WristLeft].Position.X, joints[JointType.WristLeft].Position.Y, joints[JointType.WristLeft].Position.Z);
            lElbowCoords = new Vector3D(joints[JointType.ElbowLeft].Position.X, joints[JointType.ElbowLeft].Position.Y, joints[JointType.ElbowLeft].Position.Z);
            lArmVector = Vector3D.Subtract(lWristCoords, lElbowCoords);
            //Console.WriteLine("Evg left: " + lWristCoords.ToString());
        }

        private static bool isIntersecting(Vector3D hotspotCoords, Vector3D armVector, Vector3D elbowCoords, double angle)
        {
            Vector3D l = armVector;
            l.Normalize();
            Vector3D o = elbowCoords;
            Vector3D c = hotspotCoords;
            Vector3D oc = Vector3D.Subtract(c, o);
            double absoc = Math.Sqrt(Vector3D.DotProduct(oc, oc));
            double r = Math.Tan(angle * 2 * Math.PI / 360) * absoc;
            double dist = (Vector3D.DotProduct(l, oc) - absoc + r * r);
            //Console.WriteLine("Evg: " + dist.ToString());
            //Console.WriteLine("Evg: " + rWristCoords.ToString());
            //Console.WriteLine("Right arm: "  + rWristCoords.ToString());
            if (dist < 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public class Grip
        {
            public ulong trackingId;
            public int hand; // 1 -> left, 2 -> right
            public Vector3D startCoords;
            public Vector3D prevCoords;
            public Hotspot hotspot;
            public Vector3D startVect;

            public Grip(ulong trackingId, int hand, Vector3D startCoords, Hotspot hotspot, Vector3D startVect)
            {
                this.trackingId = trackingId;
                this.hand = hand;
                this.startCoords = startCoords;
                this.prevCoords = startCoords;
                this.hotspot = hotspot;
                this.startVect = startVect;
            }
        }

        public class Hotspot
        {
            public Vector3D coords;
            public Interface iface;

            public Hotspot(Vector3D coords, Interface iface)
            {
                this.coords = coords;
                this.iface = iface;
            }
        }

        public class Interface
        {
            public string type;
            public int id;
            public SolidColorBrush brush;
            public int hue;
            public double brightness = 1.0;
            private Stopwatch watch = new Stopwatch();
            private VolumeControls volumeControls;
            int curVol;
            private TcpClient socket;
            private NetworkStream stream;
            private int last = 40055;


            public void pickup(IReadOnlyDictionary<JointType, Joint> joints)
            {
                Console.WriteLine("Picked up iterface type " + type);
                if (type == "record")
                {
                    if (isPointingToSky(joints))
                    {
                        Console.WriteLine("** Record mode entered **");
                        recorder.record();
                    }
                    else
                    {
                        Console.WriteLine("Not pointing to sky");
                    }

                }
                else if (type == "color")
                {
                    socket = new TcpClient();
                    socket.NoDelay = true;
                    socket.Connect("192.168.2.177", 50007);
                    Console.WriteLine("Socket connected");
                    stream = socket.GetStream();
                    watch.Start();
                }
            }

            public void RespCallback(IAsyncResult result)
            {

            }

            public void onChange(double x, double y)
            {
                if (type == "color")
                {
                    int sx = Convert.ToInt32(this.hue + 500 * x);
                    brush = new SolidColorBrush(ColorFromHSV(sx, 1.0, 1.0));
                    brightness = y * 1.2 + 0.3;

                    int intBrightness = (int)(brightness * 65535);
                    if (watch.ElapsedMilliseconds > 100 && (Math.Abs(last - intBrightness))>400)
                    {
                        intBrightness = intBrightness < 0 ? 0 : intBrightness;
                        intBrightness = intBrightness > 65535 ? 65535 : intBrightness;

                        UInt16 smallBright = Convert.ToUInt16(intBrightness);

                        String brightString = Convert.ToString(smallBright);
                        while (brightString.Length < 5)
                        {
                            brightString = "0" + brightString;
                        }


                        byte[] outstream = Encoding.ASCII.GetBytes(brightString);

                        stream.Write(outstream, 0, outstream.Length);
                        //stream.Flush();
                        last = intBrightness;
                        Console.WriteLine("Brightness: " + brightness.ToString() + " " + Encoding.Default.GetString(outstream) + " " + outstream.Length);
                        watch.Restart();
                    }

                }
                else if (type == "sound")
                {

                    int volumeChange = 0;
                    int volumeTimes = 0;
                    double angle = Math.Atan(x / y);

                    double scale;

                    if (Math.Abs(x) > Math.Abs(y))
                    {
                        scale = x;
                    }
                    else
                    {
                        scale = y;
                    }

                    // it's in the y (vertical) direction
                    // do sth ...

                    volumeTimes = (int)Math.Floor(scale * 100.0);
                    volumeChange = volumeTimes - curVol;

                    for (int i = 1; i <= Math.Abs(volumeChange); i++)
                    {
                        if (volumeChange > 0)
                        {
                            volumeControls.VolUp();
                        }
                        else if (volumeChange < 0)
                        {
                            volumeControls.VolDown();
                        }
                    }
                    curVol = volumeTimes;
                }
            }

            public void drop()
            {
                Console.WriteLine("Dropped a " + type);

                if (type == "color")
                {
                    stream.Close();
                    socket.Close();
                    watch.Reset();
                }
            }

            public Interface(string type, int id)
            {
                this.type = type;
                this.id = id;
                if (type == "color")
                {
                    this.hue = 180;
                    brush = new SolidColorBrush(ColorFromHSV(this.hue, 1.0, 1.0));
                }
                else if (type == "sound")
                {
                    this.volumeControls = new VolumeControls();
                    curVol = 0;
                }
            }
        }

        public static Color ColorFromHSV(double hue, double saturation, double value)
        {
            int hi = Convert.ToInt32(Math.Floor(hue / 60)) % 6;
            double f = hue / 60 - Math.Floor(hue / 60);

            value = value * 255;
            byte v = Convert.ToByte(value);
            byte p = Convert.ToByte(value * (1 - saturation));
            byte q = Convert.ToByte(value * (1 - f * saturation));
            byte t = Convert.ToByte(value * (1 - (1 - f) * saturation));

            if (hi == 0)
                return Color.FromArgb(255, v, t, p);
            else if (hi == 1)
                return Color.FromArgb(255, q, v, p);
            else if (hi == 2)
                return Color.FromArgb(255, p, v, t);
            else if (hi == 3)
                return Color.FromArgb(255, p, q, v);
            else if (hi == 4)
                return Color.FromArgb(255, t, p, v);
            else
                return Color.FromArgb(255, v, p, q);
        }


        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        public class VolumeControls
        {
            private const int APPCOMMAND_VOLUME_MUTE = 0x80000;
            private const int APPCOMMAND_VOLUME_UP = 0xA0000;
            private const int APPCOMMAND_VOLUME_DOWN = 0x90000;
            private const int WM_APPCOMMAND = 0x319;

            [DllImport("user32.dll")]
            public static extern IntPtr SendMessageW(IntPtr hWnd, int Msg, IntPtr wParam, IntPtr lParam);

            public void Mute()
            {
                SendMessageW(new WindowInteropHelper(Application.Current.MainWindow).Handle, WM_APPCOMMAND, new WindowInteropHelper(Application.Current.MainWindow).Handle,
                    (IntPtr)APPCOMMAND_VOLUME_MUTE);
            }

            public void VolDown()
            {
                SendMessageW(new WindowInteropHelper(Application.Current.MainWindow).Handle, WM_APPCOMMAND, new WindowInteropHelper(Application.Current.MainWindow).Handle,
                    (IntPtr)APPCOMMAND_VOLUME_DOWN);
            }

            public void VolUp()
            {
                SendMessageW(new WindowInteropHelper(Application.Current.MainWindow).Handle, WM_APPCOMMAND, new WindowInteropHelper(Application.Current.MainWindow).Handle,
                    (IntPtr)APPCOMMAND_VOLUME_UP);
            }
        }
    }
}

       /* private const string Port = "56700";
        private static readonly ILifxNetwork _network = LifxNetworkFactory.Instance;
        private static Socket _socket;
        


        static void LifxNetworkService()
        {
            _socket = new Socket(SocketType.Dgram, ProtocolType.Udp);
            _socket.DontFragment = true;
            
            _socket. += HandleIncomingMessages;

            var connectionProfile = NetworkInformation.GetConnectionProfiles().FirstOrDefault(
                p => p.IsWlanConnectionProfile && p.GetNetworkConnectivityLevel() != NetworkConnectivityLevel.None);

            if (connectionProfile != null)
            {
                _socket.BindServiceNameAsync(Port, connectionProfile.NetworkAdapter);
            }

            _network.RegisterSender(SendCommand);
        }

        private static async void SendCommand(IGateway gateway, byte[] data)
        {
            using (var stream = await _socket.GetOutputStreamAsync(new HostName(IpProvider.BroadcastAddress), Port))
            {
                await stream.WriteAsync(data.AsBuffer(0, data.Length));
            }
        }
        
        private static void HandleIncomingMessages(Socket sender, DatagramSocketMessageReceivedEventArgs e)
        {
            var address = e.RemoteAddress.ToString();

            using (var reader = e.GetDataReader())
            {
                var data = reader.DetachBuffer().ToArray();
                _network.ReceivedPacket(address, data);
            }

}
    }
}*/
