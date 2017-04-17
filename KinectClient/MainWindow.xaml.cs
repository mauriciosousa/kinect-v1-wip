using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Xml.Linq;
using System.Xml.XPath;
using System.IO;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;


using System.Threading;
using System.Runtime.Serialization.Formatters.Binary;
using System.Diagnostics;

namespace KinectClient
{

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Class Instance Variables

        /// <summary>
        /// Stores the ID assigned to the Kinect sensor
        /// </summary>
        private String KinectID;



        /// <summary>
        /// Kinect sensor
        /// </summary>
        private KinectSensor _sensor;

        /// <summary>
        /// Renders received skeleton frames
        /// </summary>
        private SkeletonRenderer _skeletonRenderer;

        /// <summary>
        /// Renders received depth frames
        /// </summary>
        private DepthRenderer _depthRenderer;

        /// <summary>
        /// Connection ready flag
        /// </summary>
        private bool connectionReady = false;

        /// <summary>
        /// Start sending flag
        /// </summary>
        private bool send = false;

        /// <summary>
        /// Standby timer 
        /// </summary>
        System.Windows.Threading.DispatcherTimer standbyTimer = new System.Windows.Threading.DispatcherTimer();

        /// <summary>
        /// Stores the location of the Kinect client
        /// </summary>
        private Point? _Location;
        private Point? Location
        {
            set
            {
                _Location = value;
                UpdateConfigurationFile();
            }

            get
            {
                return _Location;
            }
        }

        /// <summary>
        /// Stores the orientation of the Kinect client
        /// </summary>
        private Double? _Orientation;
        private Double? Orientation
        {
            set
            {
                _Orientation = value;
                UpdateConfigurationFile();
            }
            get
            {
                return _Orientation;
            }
        }

        /// <summary>
        /// The Directory of where the config file will be saved
        /// </summary>
        private const string CONFIGFILEDIRECTORY = "../../config/";

        /// <summary>
        /// The name of the config file
        /// </summary>
        private const string CONFIGFILELOCATION = CONFIGFILEDIRECTORY + "config.txt";

        #endregion

        #region WalkingInPlace Instance Variables

        private UdpBroadcast _udp;

        private WalkingInPlace _wip;



        private bool inSession;

        private Timeout _time;
        private long _currentTime;

        private string _veAddress;
        private int _vePort;


        private FileWriter MasterLogFile;

        private bool WipActive;
        private int WipActiveState;

        private string _timestamp;

        private bool _sendMessages;

        #endregion

        #region Constructor
        public MainWindow()
        {
            //KinectID = "Local Kinect 2";// System.Environment.MachineName;
            KinectID = "Wip Kinect";//KinectNameServer.getName(System.Environment.MachineName);

            TestKinectAvailability();

            //access saved location and orientation data from config.txt file
            LoadConfigurationFile();

            

            // Initialization
            InitializeComponent();
            InitializeKinect();

            

            // Event Handlers
            Closing += new System.ComponentModel.CancelEventHandler(MainWindow_Closing);
            kinectOffRadioButton.Checked += new RoutedEventHandler(kinectOffRadioButton_Checked);
            kinectOnRadioButton.Checked += new RoutedEventHandler(kinectOnRadioButton_Checked);

            _timestamp = DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss-fff");

            _time = new Timeout(1000);
            

            //Wip Constructer
            //_wip_feet_spine = new WalkingInPlace("wip_feet_spine_" + _timestamp + ".csv");
            //_wip_feet_shoulder = new WalkingInPlace("wip_feet_shoulder_" + _timestamp + ".csv");
            //_wip_feet_hip = new WalkingInPlace("wip_feet_hip_" + _timestamp + ".csv");

            //_wip_ankles_spine = new WalkingInPlace("wip_ankles_spine_" + _timestamp + ".csv");
            //_wip_ankles_shoulder = new WalkingInPlace("wip_ankles_shoulder_" + _timestamp + ".csv");
            //_wip_ankles_hip = new WalkingInPlace("wip_ankles_hip_" + _timestamp + ".csv");
            //_wip_ankles_hip.WalkingInPlaceEvent += new WalkingInPlaceEventHandler(_wip_ankles_hip_WalkingInPlaceEvent);

            //_wip_knees_spine = new WalkingInPlace("wip_knees_spine_" + _timestamp + ".csv");
            //_wip_knees_shoulder = new WalkingInPlace("wip_knees_shoulder_" + _timestamp + ".csv");
            
            
            _wip = new WalkingInPlace("wip_knees_hip_" + _timestamp + ".csv");       
            _wip.WalkingInPlaceEvent += new WalkingInPlaceEventHandler(_wip_WalkingInPlaceEvent);
            _wip.OutputEnabled = true;

            WipActive = false;
            WipActiveState = 0;



            _veAddress = "172.20.41.50";
            _vePort = 8881;

            inSession = false;

            

            string wipXmlConfigFile = "../../config/WipConfig.xml"; 
            if (File.Exists(wipXmlConfigFile))
            {
                Debug.WriteLine("[WalkingInPlace] Reading from xml file");

                try
                {
                    XPathDocument xml = new XPathDocument(wipXmlConfigFile);

                    foreach (XPathNavigator child in xml.CreateNavigator().Select("Wip/VirtualEnvironment"))
                    {
                        _veAddress = child.SelectSingleNode("Address").Value;
                        _vePort = int.Parse(child.SelectSingleNode("Port").Value);
                        _wip.VeFrameRate = int.Parse(child.SelectSingleNode("FrameRate").Value);
                    }

                    foreach (XPathNavigator child in xml.CreateNavigator().Select("Wip/Foot"))
                    {
                        _wip.PositionThreshold = float.Parse(child.SelectSingleNode("PositionThreshold").Value);
                        _wip.StepInitSpeedThreshold = float.Parse(child.SelectSingleNode("StepInitSpeed").Value);
                    }

                    foreach (XPathNavigator child in xml.CreateNavigator().Select("Wip/Velocity"))
                    {
                        _wip.InitialVelocity = float.Parse(child.SelectSingleNode("Initial").Value);
                        _wip.MaxVelocity = float.Parse(child.SelectSingleNode("Max").Value);
                    }

                    foreach (XPathNavigator child in xml.CreateNavigator().Select("Wip/Orientation"))
                    {
                        _wip.MinRotationThreshold = float.Parse(child.SelectSingleNode("MinRotationThreshold").Value);
                        string algorithm = child.SelectSingleNode("Algorithm").Value;
                        if (algorithm == "Exponential")
                        {
                            _wip.computeAngularVelocityMode = ComputeAngularVelocityMode.EXPONENTIAL;
                            orientationExponentialRadioButton.IsChecked = true;
                        }
                        else if (algorithm == "Time")
                        {
                            _wip.computeAngularVelocityMode = ComputeAngularVelocityMode.TIME;
                            orientationTimeRadioButton.IsChecked = true;
                        }
                        else
                        {
                            _wip.computeAngularVelocityMode = ComputeAngularVelocityMode.LINEAR;
                            orientationLinearRadioButton.IsChecked = true;
                        }

                    }
                }
                catch (Exception e)
                {
                    Debug.WriteLine("[Xml] " + e.Message);
                }

            }
            else
            {
                Debug.WriteLine("[WalkingInPlace] No xml file, using hardcoded values");
            }


            // Virtual Environment
            addressTextBox.Text = _veAddress;
            portTextBox.Text = "" + _vePort;
            framerateTextBox.Text = "" + _wip.VeFrameRate;
            // Foot
            positionThresholdTextBox.Text = "" + _wip.PositionThreshold;
            stepInitSpeedThresholdTextBox.Text = "" + _wip.StepInitSpeedThreshold;

            //Velocity
            initialVelocityTextBox.Text = "" + _wip.InitialVelocity;
            maxVelocityTextBox.Text = "" + _wip.MaxVelocity;

            //Orientation
            minRotationThresholdTextBox.Text = "" + _wip.MinRotationThreshold;

            _udpConnect();
        }
        #endregion

        #region Wip Event Handlers
        

        void _wip_WalkingInPlaceEvent(object sender, WalkingInPlaceEventArgs e)
        {

            if (WipActive)
            {
                this.stateTextBox.Text = "  Wip State:   Active";
                this.stateTextBox.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom("#FF42962A"));
            }
            else
            {
                this.stateTextBox.Text = "  Wip State:   Not Active (do activation gesture)";
                this.stateTextBox.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom("#FFBA3535"));
            }


            if (_udp != null)
            {
                string v = ("" + e.Velocity).Replace(',', '.');
                string aOffset = ("" + e.AngularOffset).Replace(',', '.');

                

                if (WipActive)
                {
                    _udp.send("wipevent/" + WipActive + "/velocity=" + v + "/angularoffset=" + aOffset);
                    dashboardVelocityLabel.Text = "" + Math.Round(e.Velocity, 3) + " m/s";
                    dashboardOrientationLabel.Text = "" + Math.Round((e.AngularOffset * _wip.VeFrameRate), 3);
                }
                else
                {
                    _udp.send("wipevent/" + WipActive + "/velocity=0/angularoffset=0");
                    dashboardVelocityLabel.Text = "0 m/s";
                    dashboardOrientationLabel.Text = "0";
                }
            }
        }
        #endregion

        #region Kinect Initialization
        private void InitializeKinect()
        {
            foreach (KinectSensor potentialSensor in KinectSensor.KinectSensors)
            {
                _sensor = potentialSensor;
                break;
            }

            // Check if a Kinect sensor was discovered
            if (_sensor != null)
            {
                _sensor.SkeletonStream.Enable();
                _sensor.DepthStream.Enable();
                _sensor.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(_sensor_DepthFrameReady);
                _sensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(_sensor_SkeletonFrameReady);

                try 
                { 
                    _sensor.Start();

                    // Change Kinect status
                    //this.kinectStatusBarText.Text = "Kinect Status: Ready";
                    kinectOnRadioButton.IsChecked = true;
                }

                catch (IOException) 
                { 
                    _sensor = null; 
                }
            }
            
            if(_sensor == null)
            {
                // Change Kinect status
                //this.kinectStatusBarText.Text = "Kinect Status: No Kinect Ready Found!";
                kinectOffRadioButton.IsChecked = true;
            }
        }

        void _sensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            this.Dispatcher.Invoke(new Action(delegate()
            {
                _depthRenderer = new DepthRenderer(depthImage, e, _sensor);
            }));
        }


        private int _lastUserID = 0;
        void _sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    this.Dispatcher.Invoke(new Action(delegate()
                    {
                        _skeletonRenderer = new SkeletonRenderer(skeletonImage, e, _sensor);
                    }));

                    #region WalkingInPlace Update
                    Skeleton [] wipSkeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(wipSkeletons);
                    
                    if (wipSkeletons.Length != 0)
                    {
                        Skeleton wipUser = null;
                        float AbsoluteDistance = 50000f;
                        foreach (Skeleton wipSkel in wipSkeletons)
                        {
                            if (wipSkel.TrackingState == SkeletonTrackingState.Tracked)
                            {
                                float skelDistance = wipSkel.Joints[JointType.HipCenter].Position.Z;
                                if (skelDistance < AbsoluteDistance)
                                {
                                    AbsoluteDistance = skelDistance;
                                    wipUser = wipSkel;
                                }
                            }
                        }

                        if (wipUser != null)
                        {

                            
                            

                            float head = wipUser.Joints[JointType.Head].Position.Y;
                            float rightHand = wipUser.Joints[JointType.HandRight].Position.Y;
                            float leftHand = wipUser.Joints[JointType.HandLeft].Position.Y;


                            Joint leftHip = wipUser.Joints[JointType.HipLeft];
                            Joint rightHip = wipUser.Joints[JointType.HipRight];

                            float leftHipAngle = (float)Math.Atan(leftHip.Position.X / leftHip.Position.Z);
                            float rightHipAngle = (float)Math.Atan(rightHip.Position.X / rightHip.Position.Z);
                            float newAngle = (leftHipAngle + rightHipAngle) / 2;



                            if (WipActiveState == 0 && leftHand > head && rightHand > head)
                            {
                                WipActiveState = 1;
                                WipActive = !WipActive;
                                _wip.setUserHeight((float) userHeight(wipUser));
                            }
                            if (WipActiveState == 1 && leftHand < head && rightHand < head)
                            {
                                WipActiveState = 0;
                            }


                            if (_lastUserID > 0)
                            {
                                if (_lastUserID != wipUser.TrackingId)
                                {
                                    WipActiveState = 0;
                                    WipActive = false;
                                }
                            }
                            _lastUserID = wipUser.TrackingId;
                            



                            float footLeft = wipUser.Joints[JointType.FootLeft].Position.Y;
                            float footRight = wipUser.Joints[JointType.FootRight].Position.Y;

                            float ankleLeft = wipUser.Joints[JointType.AnkleLeft].Position.Y;
                            float ankleRight = wipUser.Joints[JointType.AnkleRight].Position.Y;

                            float kneeLeft = wipUser.Joints[JointType.KneeLeft].Position.Y;
                            float kneeRight = wipUser.Joints[JointType.KneeRight].Position.Y;

                            


                            Vector4 hipCenterOrientationQuaternionAbsolute = wipUser.BoneOrientations[JointType.HipCenter].AbsoluteRotation.Quaternion;
                            Vector4 hipCenterOrientationQuaternionHierarchical = wipUser.BoneOrientations[JointType.HipCenter].HierarchicalRotation.Quaternion;
                            float[] hipCenterOrientationEulerAbsolute = QuaternionToEuler.toEuler(hipCenterOrientationQuaternionAbsolute);
                            float[] hipCenterOrientationEulerHierarchical = QuaternionToEuler.toEuler(hipCenterOrientationQuaternionHierarchical);

                            Vector4 spineCenterOrientationQuaternionAbsolute = wipUser.BoneOrientations[JointType.Spine].AbsoluteRotation.Quaternion;
                            Vector4 spineCenterOrientationQuaternionHierarchical = wipUser.BoneOrientations[JointType.Spine].HierarchicalRotation.Quaternion;
                            float[] spineCenterOrientationEulerAbsolute = QuaternionToEuler.toEuler(spineCenterOrientationQuaternionAbsolute);
                            float[] spineCenterOrientationEulerHierarchical = QuaternionToEuler.toEuler(spineCenterOrientationQuaternionHierarchical);

                            Vector4 shoulderCenterOrientationQuaternionAbsolute = wipUser.BoneOrientations[JointType.ShoulderCenter].AbsoluteRotation.Quaternion;
                            Vector4 shoulderCenterOrientationQuaternionHierarchical = wipUser.BoneOrientations[JointType.ShoulderCenter].HierarchicalRotation.Quaternion;
                            float[] shoulderCenterOrientationEulerAbsolute = QuaternionToEuler.toEuler(shoulderCenterOrientationQuaternionAbsolute);
                            float[] shoulderCenterOrientationEulerHierarchical = QuaternionToEuler.toEuler(shoulderCenterOrientationQuaternionHierarchical);

                            



                            float rightDiff = 0f;
                            float leftDiff = 0f;

                            if (kneeLeft > kneeRight)
                            {
                                leftDiff = kneeLeft - kneeRight;
                            }
                            else
                            {
                                rightDiff = kneeRight - kneeLeft;
                            }


                            float _userHeight = (float) userHeight(wipUser);

                            _wip.update(rightDiff, leftDiff, hipCenterOrientationEulerAbsolute[1]);
                            dashboardLeftFootHeight.Text = "" + Math.Round(leftDiff, 3);
                            dashboardRightFootHeight.Text = "" + Math.Round(rightDiff, 3);
                           
                            
                        }

                    }
                    #endregion

                }
            }
        }
        #endregion

        #region User Height
        public double boneLength(Joint p1, Joint p2)
        {
            return Math.Sqrt(
                Math.Pow(p1.Position.X - p2.Position.X, 2) +
                Math.Pow(p1.Position.Y - p2.Position.Y, 2) +
                Math.Pow(p1.Position.Z - p2.Position.Z, 2));
        }

        public double boneLength(params Joint[] joints)
        {
            double length = 0;

            for (int index = 0; index < joints.Length - 1; index++)
            {
                length += boneLength(joints[index], joints[index + 1]);
            }


            return length;
        }

        public int NumberOfTrackedJoints(params Joint[] joints)
        {
            int trackedJoints = 0;

            foreach (var joint in joints)
            {
                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    trackedJoints++;
                }
            }


            return trackedJoints;
        }

        public double userHeight(Skeleton skeleton)
        {
            const double HEAD_DIVERGENCE = 0.1;

            var head = skeleton.Joints[JointType.Head];
            var neck = skeleton.Joints[JointType.ShoulderCenter];
            var spine = skeleton.Joints[JointType.Spine];
            var waist = skeleton.Joints[JointType.HipCenter];
            var hipLeft = skeleton.Joints[JointType.HipLeft];
            var hipRight = skeleton.Joints[JointType.HipRight];
            var kneeLeft = skeleton.Joints[JointType.KneeLeft];
            var kneeRight = skeleton.Joints[JointType.KneeRight];
            var ankleLeft = skeleton.Joints[JointType.AnkleLeft];
            var ankleRight = skeleton.Joints[JointType.AnkleRight];
            var footLeft = skeleton.Joints[JointType.FootLeft];
            var footRight = skeleton.Joints[JointType.FootRight];


            // Find which leg is tracked more accurately.
            int legLeftTrackedJoints = NumberOfTrackedJoints(hipLeft, kneeLeft, ankleLeft, footLeft);
            int legRightTrackedJoints = NumberOfTrackedJoints(hipRight, kneeRight, ankleRight, footRight);


            double legLength = legLeftTrackedJoints > legRightTrackedJoints ? boneLength(hipLeft, kneeLeft, ankleLeft, footLeft) : boneLength(hipRight, kneeRight, ankleRight, footRight);


            return boneLength(head, neck, spine, waist) + legLength + HEAD_DIVERGENCE;
        }

        #endregion
        #region Helper Functions
        /// <summary>
        /// Converts an arbitrary object type to a Byte array
        /// </summary>
        /// <param name="obj">Object to be converted</param>
        /// <returns>Byte array representation of the passed object</returns>
        private byte[] objectToByteArray(Object obj)
        {
            if (obj == null)
                return null;
            BinaryFormatter bf = new BinaryFormatter();
            MemoryStream ms = new MemoryStream();
            bf.Serialize(ms, obj);

            return ms.ToArray();
        }

        /// <summary>
        /// Runs a Kinect check and exits if none detected
        /// </summary>
        private void TestKinectAvailability()
        {
            // Checks to see how many Kinects are connected to the system. If none then exit.
            if (KinectSensor.KinectSensors.Count == 0)
            {
                System.Windows.Forms.MessageBox.Show("No Kinect detected. Please plug in a Kinect and restart the program", "No Kinect Detected!");
                Environment.Exit(0);
            }
        }
        #endregion



     

        #region Window Event Handlers
        /// <summary>
        /// Handles clean exiting of the program 
        /// </summary>
        /// <param name="sender">Window object</param>
        /// <param name="e">Cancel event args</param>
        void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            
            if (_sensor != null) { _sensor.Stop(); }
            Environment.Exit(0);
        }
        #endregion

        #region Kinect Tilt Angle Control
        /// <summary>
        /// Captures the start of slider move
        /// </summary>
        /// <param name="sender">Slider object</param>
        /// <param name="e">Mouse button event args</param>
        private void Slider_MouseDown(object sender, MouseButtonEventArgs e)
        {
            var fe = sender as FrameworkElement;

            if (null != fe)
            {
                if (fe.CaptureMouse())
                {
                    e.Handled = true;
                }
            }
        }

        /// <summary>
        /// Changes the tilt angle of the sensor
        /// </summary>
        /// <param name="sender">Slider object</param>
        /// <param name="e">Mouse button event args</param>
        private void Slider_MouseUp(object sender, MouseButtonEventArgs e)
        {
            var fe = sender as FrameworkElement;

            if (null != fe)
            {
                if (fe.IsMouseCaptured)
                {
                    fe.ReleaseMouseCapture();
                    e.Handled = true;
                }

                Int16 newTiltAngle = Int16.Parse(tiltAngleTextBox.Text);

                try
                {
                    _sensor.ElevationAngle = newTiltAngle;
                }
                catch (InvalidOperationException exception)
                {
                    System.Console.WriteLine(exception.Message);
                }
            }
        }

        /// <summary>
        /// Changes tilt angle as the slider moves
        /// </summary>
        /// <param name="sender">Slider object</param>
        /// <param name="e">Mouse event args</param>
        private void Slider_MouseMove(object sender, MouseEventArgs e)
        {
            var fe = sender as FrameworkElement;

            if (null != fe)
            {
                if (fe.IsMouseCaptured)
                {
                    var position = Mouse.GetPosition(this.SliderTrack);
                    int newAngle = -27 + (int)Math.Round(54.0 * (this.SliderTrack.ActualHeight - position.Y) / this.SliderTrack.ActualHeight);

                    if (newAngle < -27)
                    {
                        newAngle = -27;
                    }
                    else if (newAngle > 27)
                    {
                        newAngle = 27;
                    }
                    RotateTransform rt = new RotateTransform(-2 * newAngle);
                    SliderArrow.RenderTransform = rt;
                    tiltAngleTextBox.Text = newAngle.ToString();
                }
            }
        }
        #endregion

 


        #region Kinect Status Controls

        /// <summary>
        /// Turns the Kinect on
        /// </summary>
        /// <param name="sender">Radio button object</param>
        /// <param name="e">Routed event args</param>
        void kinectOnRadioButton_Checked(object sender, RoutedEventArgs e)
        {
            if (!_sensor.IsRunning)
            {
                InitializeKinect();
            }
        }

        /// <summary>
        /// Turns the Kinect off
        /// </summary>
        /// <param name="sender">Radio button object</param>
        /// <param name="e">Routed event args</param>
        void kinectOffRadioButton_Checked(object sender, RoutedEventArgs e)
        {
            if (_sensor.IsRunning)
            {
                try
                {
                    _sensor.Stop();
                }
                catch (Exception exception)
                {
                    System.Console.WriteLine(exception.Message);
                }
            }
        }
        #endregion        

        #region Saving and accessing Location

        private void LoadConfigurationFile()
        {
            if (!Directory.Exists(CONFIGFILEDIRECTORY))
            {
                Directory.CreateDirectory(CONFIGFILEDIRECTORY);
            }

            if (!File.Exists(CONFIGFILELOCATION))
            {
                System.IO.File.Create(CONFIGFILELOCATION).Close();
            }
            else
            {
                string[] lines = File.ReadAllLines(CONFIGFILELOCATION);
                ParseConfigurationFile(lines);
            }
        }

        private void ParseConfigurationFile(string[] lines)
        {
            foreach (string line in lines)
            {
                string[] s = line.Split(':');

                if (s.Length == 2)
                {
                    switch (s[0])
                    {
                        case "location":

                            // Split on space to get multiple parameters per line
                            string[] location = s[1].Split(' ');

                            try
                            {
                                if (location.Length == 2)
                                {
                                    Location = (Point?) new System.Windows.Point(Convert.ToDouble(location[0]), Convert.ToDouble(location[1]));
                                }
                            }
                            catch
                            {
                                Console.Write("Configuration File has invalid data for Location field");
                            }

                            break;
                        case "orientation":
                            try
                            {
                                Orientation = Convert.ToDouble(s[1]);
                            }
                            catch
                            {
                                Console.Write("Configuration File has invalid data for Orientation field");
                            }
                            break;
                    }
                }
            }
        }

        private void UpdateConfigurationFile()
        {
            List<string> outputStrings = new List<string>();

            if (Location != null)
            {
                outputStrings.Add("location:" + Location.Value.X + " " + Location.Value.Y);
            }
            if (Orientation != null)
            {
                outputStrings.Add("orientation:" + Orientation);
            }

            string[] outputString = outputStrings.ToArray();

            try
            {
                File.WriteAllLines(CONFIGFILELOCATION, outputString);
            }
            catch (IOException exception)
            {
                System.Console.WriteLine(exception.Message);
            }

        }

        #endregion

        private void positionThresholdTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            float newPositionThreshold = float.Parse(positionThresholdTextBox.Text);
            _wip.PositionThreshold = newPositionThreshold;
            Debug.WriteLine("[Position Threshold] changed to " + _wip.PositionThreshold);
        }

        private void framerateTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            int newFrameRate = int.Parse(framerateTextBox.Text);
            _wip.VeFrameRate = newFrameRate;
            Debug.WriteLine("[VE Frame Rate] changed to " + _wip.VeFrameRate);
        }

        private void initialVelocityTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            float newInitialVelocity = float.Parse(initialVelocityTextBox.Text);
            _wip.InitialVelocity = newInitialVelocity;
            Debug.WriteLine("[wip.InitialVelocity] changed to " + _wip.InitialVelocity);
        }

        private void maxVelocityTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            float newMaxVelocity = float.Parse(maxVelocityTextBox.Text);
            _wip.MaxVelocity = newMaxVelocity;
            Debug.WriteLine("[wip.MaxVelocity] changed to " + _wip.MaxVelocity);
        }

        private void minRotationThresholdTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            float newMinRotationThreshold = float.Parse(minRotationThresholdTextBox.Text);
            _wip.MinRotationThreshold = newMinRotationThreshold;
            Debug.WriteLine("[wip.MinRotationThreshold] changed to " + _wip.MinRotationThreshold);
        }

        

        private void stepInitSpeedThresholdTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            float newStepInitThreshold = float.Parse(stepInitSpeedThresholdTextBox.Text);
            _wip.StepInitSpeedThreshold = newStepInitThreshold;
            Debug.WriteLine("[wip.stepInitSpeedThreshold] changed to " + _wip.StepInitSpeedThreshold);
        }

        private void _udpConnect()
        {

            try
            {
                if (addressTextBox.Text != "" && portTextBox.Text != "")
                {
                    _veAddress = addressTextBox.Text;
                    _vePort = int.Parse(portTextBox.Text);
                    _udp = new UdpBroadcast(_veAddress, _vePort);
                    Debug.WriteLine("[UDP] broadcasting to " + _veAddress + ":" + _vePort);
                }
                else
                {
                    throw new Exception();
                }
            }
            catch (Exception e)
            {
                _udp = null;
                Debug.WriteLine("[UDP][ERROR] wrong IP address or port"); 
            }
        }

        private double getAngle(SkeletonPoint P1, SkeletonPoint P2, SkeletonPoint P3)
        {
            SkeletonPoint a = new SkeletonPoint();
            a.X = P2.X - P1.X;
            a.Y = P2.Y - P1.Y;
            a.Z = P2.Z - P1.X;

            SkeletonPoint b = new SkeletonPoint();
            b.X = P3.X - P1.X;
            b.Y = P3.Y - P1.Y;
            b.Z = P3.Z - P1.X;

            SkeletonPoint axb = new SkeletonPoint();
            axb.X = a.Y * b.Z - a.Z * b.Y;
            axb.Y = - (a.X * b.Z - a.Z * b.X);
            axb.Z = a.X * b.Y - a.Y * b.X;

            double A = Math.Sqrt(axb.X * axb.X + axb.Y * axb.Y + axb.Z * axb.Z);
            double B = Math.Sqrt(0 + 0 + (1) * (1));
            double AB = A * B;

            double r = (axb.X * 0 + axb.Y * 0 + axb.Z * (1)) / AB;
            double theta = Math.Acos(r);

            return theta;
        }

        private void addressTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            _udpConnect();
        }

        private void portTextBox_LostFocus(object sender, RoutedEventArgs e)
        {
            _udpConnect();
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            _udp.send("wipevent/" + WipActive + "/velocity=0/angularoffset=0");
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            _udp.send("wipevent/" + WipActive + "/velocity=0/angularoffset=0");
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            _udp.send("veevent/" + WipActive + "/reset");
        }

        private void orientationTimeRadioButton_Checked(object sender, RoutedEventArgs e)
        {
            _wip.computeAngularVelocityMode = ComputeAngularVelocityMode.TIME;
            Debug.WriteLine("[WIP] Orientation  = " + _wip.computeAngularVelocityMode.ToString());
        }
        private void orientationLinearRadioButton_Checked(object sender, RoutedEventArgs e)
        {
            _wip.computeAngularVelocityMode = ComputeAngularVelocityMode.LINEAR;
            Debug.WriteLine("[WIP] Orientation  = " + _wip.computeAngularVelocityMode.ToString());
        }
        private void orientationExponentialRadioButton_Checked(object sender, RoutedEventArgs e)
        {
            _wip.computeAngularVelocityMode = ComputeAngularVelocityMode.EXPONENTIAL;
            Debug.WriteLine("[WIP] Orientation  = " + _wip.computeAngularVelocityMode.ToString());
        }




        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key.ToString() == "Space")
            {
                inSession = !inSession;
            }
        }

        private void sendMessagesRadioButtonON_Checked(object sender, RoutedEventArgs e)
        {
            _sendMessages = true;
        }

        private void sendMessagesRadioButtonOFF_Checked(object sender, RoutedEventArgs e)
        {
            _sendMessages = false;
        }




    }
}
