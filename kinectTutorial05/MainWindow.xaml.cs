using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
// using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Threading;
using System.Xml;
using System.Xml.Linq;
using System.Threading.Tasks;
using System.IO;


using System.ComponentModel;

using Microsoft.Kinect;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.Remoting.Metadata.W3cXsd2001;

namespace kinectKata
{
    
    public enum DisplayFrameType
    {
        Body
    }
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        // Setup interface option selection
       // private const DisplayFrameType DEFAULT_DISPLAYFRAMETYPE = DisplayFrameType.Body;

        /// Kinect Sensor
        private KinectSensor kinectSensor = null;

        // Object to write the image to show on the interface for DEPTH, COLOR and INFRARED sources
        private WriteableBitmap bitmap = null;
        private FrameDescription currentFrameDescription;
        private DisplayFrameType currentDisplayFrameType;
        // Reader to receive the information from the camera
        private MultiSourceFrameReader multiSourceFrameReader = null;



        //VARIABLES NEEDED  FOR BODY TRACKING //

        /// Control negative values of depth map: Constant for clamping Z values of camera space points
        private const float InferredZPositionClamp = 0.1f;

        //** Variables for creating and storing bones coordinates
        /// Coordinate mapper to map one type of point to another from DEPTH to COLOR
        private CoordinateMapper coordinateMapper = null;
        /// Array for the bodies
        private Body[] bodies = null;
        /// definition of bones
        private List<Tuple<JointType, JointType>> bones;
        /// List of colors for each body tracked
        private List<Pen> bodyColors;

        //*** Variables for drawing the JOINTS (points of intersection between bones): 
        /// Thickness of drawn joint lines
        private const double JointThickness = 3;
        /// Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;
        /// Brush for tracked joints
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        /// Brush for inferred joints
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        /// Pen for drawing inferred bones
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        //** Variables for drawing the HANDS:
        /// Radius of drawn hand circles
        private const double HandSize = 20;
        /// Brush for closed hands
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        /// Brush for opened hands
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        /// Brush for pointed hands
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        
        //** Variables for the Image object to import the results of the skeleton
        /// Drawing group for body rendering output
        private DrawingGroup drawingGroup;
        /// Drawing image that we will display
        private DrawingImage imageSource;
        /// Width of display (depth space)
        private int displayWidthBody;
        /// Height of display (depth space)
        private int displayHeightBody;

        private int meanCount = 0;

        private List<double> elbowL_wristL_values = new List<double>();
        private List<double> elbowR_wristR_values = new List<double>();
        private List<double> shoulderL_elbowL_values = new List<double>();
        private List<double> shoulderR_elbowR_values = new List<double>();
        private List<double> hipL_kneeL_values = new List<double>();
        private List<double> hipR_kneeR_values = new List<double>();
        private List<double> kneeL_ankleL_values = new List<double>();
        private List<double> kneeR_ankleR_values = new List<double>(); 

        Stopwatch value_record_timer = new Stopwatch();

        private int current_kata_id = 1;
        private int current_position = 1;
        private int position_hold_timer = 0;
        
        private List<List<double>> current_kata;

        private System.Timers.Timer timer;

        public void initializeCurrentKata()
        {
            this.current_kata = new List<List<double>>();

            var filename = "XMLFile1.xml";
            var currentDirectory = Directory.GetCurrentDirectory();
            var kataFilepath = System.IO.Path.Combine(currentDirectory, filename);

            XElement Xkata = XElement.Load(kataFilepath);
            IEnumerable<XElement> Xcurrent_kata = Xkata.Elements("pos");

            int i = 0;
            foreach (XElement k in Xcurrent_kata)
            {
                current_kata.Add(new List<double>());
                IEnumerable<XElement> angleList = k.Elements("angle");

                foreach (XElement a in angleList)
                {
                    var val = XmlConvert.ToDouble(a.Value.Trim());
                    current_kata[i].Add(val);
                }
                i++;
            }
        }

        public MainWindow()
        {
            // Initialize the sensor
            this.kinectSensor = KinectSensor.GetDefault();

            // Open the reader for the  frames
            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader
                (FrameSourceTypes.Body);
            // Wire handler for frame arrival - This is a later defined method 
             this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;




            // Set up display frame types:
            //SetupCurrentDisplay(DEFAULT_DISPLAYFRAMETYPE);
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription bodyDepthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.CurrentFrameDescription = bodyDepthFrameDescription;

            // get size of the scene
            this.displayWidthBody = bodyDepthFrameDescription.Width;
            this.displayHeightBody = bodyDepthFrameDescription.Height;

            // Define a bone as the line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();
            // Create the body bones
            this.defineBoneParts();

            // Populate body colors that you wish to show, one for each BodyIndex:
            this.bodyColors = new List<Pen>();
            this.bodyIndexColors();

            // We need to create a drawing group
            this.drawingGroup = new DrawingGroup();


            // We transfer the data to our Window defined in the XAML file
            this.DataContext = this;
            // Open the sensor
            this.kinectSensor.Open();

            InitializeComponent();

            // Initialize current_kata with the kata.xml file
            initializeCurrentKata();

            UpdateStatus(false,false,false,false,false);
        }

        public event PropertyChangedEventHandler PropertyChanged;
        public ImageSource ImageSource
        {
            get
            {
                return this.bitmap;
            }
        }

        public FrameDescription CurrentFrameDescription
        {
            get { return this.currentFrameDescription; }
            set
            {
                if (this.currentFrameDescription != value)
                {
                    this.currentFrameDescription = value;
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("CurrentFrameDescription"));
                    }
                }
            }
        }

        
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();


            using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                ShowBodyFrame(bodyFrame);
            }
    
        }



        private void ShowBodyFrame(BodyFrame bodyFrame)
        {
            bool dataReceived = false;
            if (bodyFrame != null)
            {

                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }

                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed/eliminated and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(this.bodies);
                dataReceived = true;
            }


            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.DarkViolet, null, new Rect(0.0, 0.0, this.displayWidthBody, this.displayHeightBody));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                // Console.WriteLine(position.Z);
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
                        }
                    }

                    // Draw only in the area visible for the camera
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidthBody, this.displayHeightBody));
                }
                // Send to our UI/Interface the created bodies to display in the Image:
                FrameDisplayImage.Source = new DrawingImage(this.drawingGroup);

            }
        }



        // ***************************************************************************//
        // ************************* BODY DATA PROCESSING **************************//

        /// <summary>
        /// Draws a body
        /// </summary>
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

        /// Draws one bone of a body (joint to joint)
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            // A bone results from the union of two joints/vertices
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            //RecordValue(joints, joint0, jointType0, joint1, jointType1);
            if (CheckCurrentKataPosition(joints, joint0, jointType0, joint1, jointType1))
            {
                position_hold_timer++;
                Console.WriteLine("Correct Position!");
            } else
            {
                position_hold_timer = 0;
            }
            if (position_hold_timer >= 30)
            {
                current_position++;
                position_hold_timer = 0;
            }


            // If we can't find either of these joints, we cannot draw them! Exit
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


         private bool CheckCurrentKataPosition(IReadOnlyDictionary<JointType, Joint> joints, Joint joint0, JointType jointType0, Joint joint1, JointType jointType1)
         {
             Joint shoulderL = joints[JointType.ShoulderLeft];
             Joint shoulderR = joints[JointType.ShoulderRight];
             Joint elbowL = joints[JointType.ElbowLeft];
             Joint elbowR = joints[JointType.ElbowRight];
             Joint wristL = joints[JointType.WristLeft];
             Joint wristR = joints[JointType.WristRight];
        
             bool shoulderL_ElbowL = false, shoulderR_ElbowR = false, elbowL_WristL = false, elbowR_WristR = false;
        
             double elbowL_wristL_Angle = GetBoneAngle(elbowL.Position.X, elbowL.Position.Y, wristL.Position.X, wristL.Position.Y);
            Console.WriteLine("elbow_wrist_L: " + elbowL_wristL_Angle);
             double shoulderL_elbowL_Angle = GetBoneAngle(shoulderL.Position.X, shoulderL.Position.Y, elbowL.Position.X, elbowL.Position.Y);
            Console.WriteLine("shoulderL_elbowL_Angle: " + shoulderL_elbowL_Angle);
             double elbowR_wristR_Angle = GetBoneAngle(elbowR.Position.X, elbowR.Position.Y, wristR.Position.X, wristR.Position.Y);
            Console.WriteLine("elbowR_wristR_Angle: " + elbowR_wristR_Angle);
             double shoulderR_elbowR_Angle = GetBoneAngle(shoulderR.Position.X, shoulderR.Position.Y, elbowR.Position.X, elbowR.Position.Y);
            Console.WriteLine("shoulderR_elbowR_Angle: " + shoulderR_elbowR_Angle);
        
             shoulderL_ElbowL = ComparePositions(shoulderL_elbowL_Angle, current_kata[current_position][0]);
             shoulderR_ElbowR = ComparePositions(shoulderR_elbowR_Angle, current_kata[current_position][1]);
             elbowL_WristL = ComparePositions(elbowL_wristL_Angle, current_kata[current_position][2]);
             elbowR_WristR = ComparePositions(elbowR_wristR_Angle, current_kata[current_position][3]);

            
            if (shoulderR_ElbowR && elbowR_WristR)
            {
                UpdateStatus(true, true, false, true, true);
                if (shoulderL_ElbowL && elbowL_WristL)
                {
                    UpdateStatus(true, true, true, true, true);
                }
            }
            else
            {
                UpdateStatus(false, false, false, false, false);
            }

            return shoulderL_ElbowL && shoulderR_ElbowR && elbowL_WristL && elbowR_WristR; 
         }

         private bool ComparePositions(double pos, double expected)
         {
            double error = 15;        
             return pos < expected + error && pos > expected - error;
         }

        private double NormalizeAngle(double angle)
        {
            if (angle >= 0) return angle;

            return angle + (2 * Math.PI);
        }
        private void RecordValue(IReadOnlyDictionary<JointType, Joint> joints, Joint joint0, JointType jointType0, Joint joint1, JointType jointType1)
        {
            

            Joint shoulderL = joints[JointType.ShoulderLeft];
            Joint shoulderR = joints[JointType.ShoulderRight];
            Joint elbowL = joints[JointType.ElbowLeft];
            Joint elbowR = joints[JointType.ElbowRight];
            Joint wristL = joints[JointType.WristLeft];
            Joint wristR = joints[JointType.WristRight];
            Joint hipL = joints[JointType.HipLeft];
            Joint hipR = joints[JointType.HipRight];
            Joint kneeL = joints[JointType.KneeLeft];
            Joint kneeR = joints[JointType.KneeRight];
            Joint ankleL = joints[JointType.AnkleLeft];
            Joint ankleR = joints[JointType.AnkleRight];

            double shoulderL_elbowL_angle = GetBoneAngle(shoulderL.Position.X, elbowL.Position.X, shoulderL.Position.Y, elbowL.Position.Y);
            double shoulderR_elbowR_angle = GetBoneAngle(shoulderR.Position.X, elbowR.Position.X, shoulderR.Position.Y, elbowR.Position.Y);
            double elbowL_wristL_angle = GetBoneAngle(elbowL.Position.X, wristL.Position.X, elbowL.Position.Y, wristL.Position.Y);
            double elbowR_wristR_angle = GetBoneAngle(elbowR.Position.X, wristR.Position.X, elbowR.Position.Y, wristR.Position.Y);
            double hipL_kneeL_angle = GetBoneAngle(hipL.Position.X, kneeL.Position.X, hipL.Position.Y, kneeL.Position.Y);
            double hipR_kneeR_angle = GetBoneAngle(hipR.Position.X, kneeR.Position.X, hipR.Position.Y, kneeR.Position.Y);
            double kneeL_ankleL_angle = GetBoneAngle(kneeL.Position.X, ankleL.Position.X, kneeL.Position.Y, ankleL.Position.Y);
            double kneeR_ankleR_angle = GetBoneAngle(kneeR.Position.X, ankleR.Position.X, kneeR.Position.Y, ankleR.Position.Y);


            shoulderL_elbowL_values.Add(shoulderL_elbowL_angle);
            shoulderR_elbowR_values.Add(shoulderR_elbowR_angle);
            elbowL_wristL_values.Add(elbowL_wristL_angle);
            elbowR_wristR_values.Add(elbowR_wristR_angle);
            hipL_kneeL_values.Add(hipL_kneeL_angle);
            hipR_kneeR_values.Add(hipR_kneeR_angle);
            kneeL_ankleL_values.Add(kneeL_ankleL_angle);
            kneeR_ankleR_values.Add(kneeR_ankleR_angle);

            if (shoulderL_elbowL_values.Count > 0)
            {
                Console.Write("shoulderL_elbowL : {0}\n" +
                "shoulderR_elbowR : {1}\n" +
                "elbowL_wristL : {2}\n" +
                "elbowR_wristR : {3}\n" +
                "hipL_kneeL : {4}\n" +
                "hipR_kneeR : {5}\n" +
                "kneeL_ankleL : {6}\n" +
                "kneeR_ankleR : {7}\n",shoulderL_elbowL_values.Average().ToString(), shoulderR_elbowR_values.Average().ToString(),
                elbowL_wristL_values.Average().ToString(), elbowR_wristR_values.Average().ToString(), hipL_kneeL_values.Average().ToString()
                ,hipR_kneeR_values.Average().ToString(), kneeL_ankleL_values.Average().ToString(), kneeR_ankleR_values.Average().ToString());

                // Thread.Sleep(1000);
            }
            
        }

        private double GetArrayMean(double[] arr)
        {
            double sum = 0;

            for (int i = 0; i < arr.Length; i++)
            {
                sum += arr[i];
            }

            return sum / arr.Length;
        }

        private double GetBoneAngle(float joint1_X, float joint2_X, float joint1_Y, float joint2_Y)
        {
            double angle;

            float deltaX = joint1_X - joint2_X;
            float deltaY = joint1_Y - joint2_Y;

            float delta = deltaY / deltaX;

            angle = Math.Atan(delta);

            return Math.Abs(angle * (180 / Math.PI));
        }
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso ergo pointing
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

        /// Draws indicators to show which edges are clipping body data
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Indigo,
                    null,
                    new Rect(0, this.displayHeightBody - ClipBoundsThickness, this.displayWidthBody, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Indigo,
                    null,
                    new Rect(0, 0, this.displayWidthBody, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Indigo,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeightBody));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Indigo,
                    null,
                    new Rect(this.displayWidthBody - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeightBody));
            }
        }

        /// <summary>
        ///  This colors are for the bodies detected by the camera, for the Kinect V2 a maximum of 6
        /// </summary>
        private void bodyIndexColors()
        {

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

        }

        /// <summary>
        /// Define which parts are connected between them
        /// </summary>
        private void defineBoneParts()
        {
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

        }



        // ***************************************************************************//
        // *************************        INTERFACE       **************************//
        private void HandleSuccessfulPosition()
        {
            Dispatcher.Invoke(() =>
            {
                SuccessMessage.Visibility = Visibility.Visible;
            });

            timer = new System.Timers.Timer(1000); 
            timer.Elapsed += OnTimerElapsed;
            timer.AutoReset = false; 

            timer.Start();

        }

        private void OnTimerElapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            timer.Stop();

            NextPosition();
        }

        private void NextPosition()
        {
        

            Dispatcher.Invoke(() =>
            {
                SuccessMessage.Visibility = Visibility.Collapsed;
                PositionName.Text = $"Position : {current_position} ";
                Uri newImageSource = new Uri($"/position_{current_position}.png", UriKind.Relative);
                image_example.Source = new BitmapImage(newImageSource);
            });
            
        }

        private void UpdateStatus(bool isTorsoOK, bool isRightArmOK, bool isLeftArmOK, bool isRightLegOK, bool isLeftLegOK)
        {
            TorsoStatus.Text = "Torse: " + (isTorsoOK ? "OK" : "KO");
            RightArmStatus.Text = "Right Arm: " + (isRightArmOK ? "OK" : "KO");
            LeftArmStatus.Text = "Left Arm: " + (isLeftArmOK ? "OK" : "KO");

            if (isRightArmOK && isLeftArmOK)
            {
                HandleSuccessfulPosition();
            }
        }

        private void Next_Click(object sender, RoutedEventArgs e)
        {
            if (current_position < 31)
            {
                current_position++;
                NextPosition();
            }
            else
            {
                NextPosition();
            }
        }

        private void Before_Click(object sender, RoutedEventArgs e)
        {
            if(current_position < 2)
            {
                NextPosition();
            }
            else {
                current_position--;
                NextPosition();
            }
        }
    }


}
