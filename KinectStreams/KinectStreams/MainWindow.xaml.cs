using Microsoft.Kinect;
using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Windows.Media.Media3D;

namespace KinectStreams
{
    /*
     * Contains all necessary methods, variables, and code to run the
     * KinectStreams application
     */
    public partial class MainWindow : Window
    {
        #region GLOBAL_VARS
        /*
         * Variable declarations - variables and objects that must be used
         * throughout the class, by multiple methods
         */
        Mode _mode = Mode.Color;
        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
        bool _displayBody = false;
        DispatcherTimer dispatcherTimer = new DispatcherTimer();
        // Timer to display the stopwatch used in all tests
        DispatcherTimer countdown; // Countdown used for balance test
        private DateTime startTime; // Time the test will start
        private double distance, startDistance; // Current distance, and starting distance for some tests
        private int curTest = 0; // Stores which test is in progress, used for if statements
        private bool balanceStart = false; // Boolean to track initial position of feet or balance test
        private bool chairStart = false; // Boolean to track initial height of mid-spine for chair-rise test
        private bool hasStood = false;
        // Boolean to track when the subject is standing up, in chair-rise/get-up-and-go tests
        private float blx = 0, bly = 0, blz = 0, brx = 0, bry = 0, brz = 0;
        // Initial positios of feet for balance test, xyz
        private float yi = 0; // Records initial height of a specific joint, used in chair-rise/get-up-and-go tests
        private int numUp = 0; // Records number of chair-rises completed in chair-rise test
        private bool startedTracking = false;
        // Boolean to detect when an aspect of the test begins (all tests except balance)
        private bool hasLeft = false;
        // Boolean to record when the subject leaves the Kinect's range in the get-up-and-go test
        private bool comingBack = false;
        // Boolean to record when the subject re-enters the Kinect's range (get-up-and-go-test)
        private double GS_STOP;
        private double BA_STEP, BA_TIME;
        private double GUAG_HEIGHT, GUAG_LEAVE, GUAG_RETURN_MAX, GUAG_RETURN_MIN, GUAG_SIT_DIST;
        private double CR_START_HEIGHT, CR_HEIGHT, CR_REPS;
        private String KINECT_PATH;

        #endregion

        #region DEFAULT_METHODS
        /*
         * Constructor to initialize main window of the KinectStreams application
         */
        public MainWindow()
        {
            InitializeComponent();
        }
        /*
         * Executed when window initially loads
         */
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // Following code will read in values from the config file
            string line;
            int curVal = 1;
            System.IO.StreamReader file =
               new System.IO.StreamReader("config.txt");
            while ((line = file.ReadLine()) != null)
            {
                if (line.StartsWith("//") || line.Equals(""))
                    curVal--;
                else if (curVal == 1)
                    GS_STOP = Convert.ToDouble(line);
                else if (curVal == 2)
                    BA_STEP = Convert.ToDouble(line);
                else if (curVal == 3)
                    BA_TIME = Convert.ToDouble(line);
                else if (curVal == 4)
                    GUAG_HEIGHT = Convert.ToDouble(line);
                else if (curVal == 5)
                    GUAG_LEAVE = Convert.ToDouble(line);
                else if (curVal == 6)
                    GUAG_RETURN_MAX = Convert.ToDouble(line);
                else if (curVal == 7)
                    GUAG_RETURN_MIN = Convert.ToDouble(line);
                else if (curVal == 8)
                    GUAG_SIT_DIST = Convert.ToDouble(line);
                else if (curVal == 9)
                    CR_START_HEIGHT = Convert.ToDouble(line);
                else if (curVal == 10)
                    CR_HEIGHT = Convert.ToDouble(line);
                else if (curVal == 11)
                    CR_REPS = Convert.ToDouble(line);
                else if (curVal == 12)
                    KINECT_PATH = line;
                curVal++;
            }
            file.Close();
            // PATH BELOW CAN BE CHANGED DEPENDING ON WHERE KINECT HAS BEEN INSTALLED
            Process.Start(@KINECT_PATH); // Starts KinectService - required to use application
            // NEXT LINES - Initialize some display settings
            camera.Width = System.Windows.SystemParameters.PrimaryScreenWidth * 2 / 3;
            camera.Height = System.Windows.SystemParameters.PrimaryScreenHeight;
            camera.HorizontalAlignment = System.Windows.HorizontalAlignment.Left;
            canvas.Height = camera.Height;
            camera.VerticalAlignment = System.Windows.VerticalAlignment.Top;
            // Initialize stopwatch, tick every millisecond
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 0, 1);
            // Initialize Kinect camera
            _sensor = KinectSensor.GetDefault();
            // Open camera and receive frame reader if sensor exists
            if (_sensor != null)
            {
                _sensor.Open();
                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            }
        }
        /*
         * Stopwatch, displays current time subtracted by the start time
         */
        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            lblTime.Content = "Time: " + ((DateTime.Now - startTime).Minutes.ToString() + ":" +
                    (DateTime.Now - startTime).Seconds.ToString() + "." + (DateTime.Now - startTime).Milliseconds.ToString());
        }
        /*
         * Executed when window is closed
         */
        private void Window_Closed(object sender, EventArgs e)
        {
            // Dispose of reader if it exists
            if (_reader != null)
            {
                _reader.Dispose();
            }
            // Remove each body from _bodies
            if (_bodies != null)
            {
                if (_bodies.Count() > 0)
                {
                    foreach (var body in _bodies)
                    {
                        _bodies.Remove(body);
                    }
                }
            }
            // Close Kinect camera
            if (_sensor != null)
            {
                _sensor.Close();
            }
        }
        /*
         * Displays and analyzes each individual frame.
         * This method contains the logic behind each test.
         */
        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            #region PRELIM_BLOCKS
            var reference = e.FrameReference.AcquireFrame(); // Holds Color, Depth, and IR frames
            // Color
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Color)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }
            // Depth
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Depth)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }
            //Infrared
            using (var frame = reference.InfraredFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (_mode == Mode.Infrared)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }
            #endregion

            #region LIVE_STREAM
            // Analyzes each individual frame
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null) // Only if frame exists
                {
                    canvas.Children.Clear(); // Clear canvas (skeletal figures)
                    _bodies = new Body[frame.BodyFrameSource.BodyCount]; // Initialize array for bodies being tracked
                    frame.GetAndRefreshBodyData(_bodies); // Retrieve bodies being tracked
                    foreach (var body in _bodies) // For each body (should only be 1 in view during tests)
                    {
                        if (body != null) // Only if body exists
                        {
                            // Draw skeleton.
                            if (_displayBody)
                            {
                                canvas.DrawSkeleton(body);
                            }
                            if (body.IsTracked) // Only if body is being tracked
                            {
                                #region LOCAL_VARS_AND_KNEE_ANGLE
                                // Initialize joints being used
                                Joint knee = body.Joints[JointType.KneeRight];
                                Joint hip = body.Joints[JointType.HipRight];
                                Joint ankle = body.Joints[JointType.AnkleRight];
                                Joint spineMid = body.Joints[JointType.SpineMid];
                                // Vectors used to determine knee joint angle
                                Vector3D v1 = new Vector3D(hip.Position.X - knee.Position.X, hip.Position.Y - knee.Position.Y, hip.Position.Z - knee.Position.Z);
                                Vector3D v2 = new Vector3D(ankle.Position.X - knee.Position.X, ankle.Position.Y - knee.Position.Y, ankle.Position.Z - knee.Position.Z);
                                // Dot product/angle of knee joint
                                double dotProd = Vector3D.DotProduct(v1, v2);
                                double angle = Math.Acos(dotProd / (v1.Length * v2.Length)) / Math.PI * 180;
                                // Position of mid-spine, used in distance
                                float x = spineMid.Position.X;
                                float y = spineMid.Position.Y;
                                float z = spineMid.Position.Z;
                                // Calculates distance of mid-spine (body) to Kinect
                                distance = Math.Sqrt(x * x + y * y + z * z);
                                #endregion

                                // Display distance of body and angle of knee joint
                                // CordBox.Content = "Distance: " + Math.Round(distance, 3).ToString() + " m";
                                // AngleBox.Content = "Angle: " + Math.Round(angle, 4).ToString() + " degrees";

                                #region 1-GAIT_SPEED
                                // Test #1: Gait-speed
                                if (dispatcherTimer.IsEnabled && curTest == 1)
                                {
                                    if (!startedTracking && body.IsTracked) // Start tracking body, initialize time/distance
                                    {
                                        startedTracking = true;
                                        startDistance = distance;
                                        startTime = DateTime.Now;
                                    }
                                    if (distance < GS_STOP) // Stop test once subject comes within 1 meter of the Kinect
                                    {
                                        dispatcherTimer.Stop();
                                        DateTime endTime = DateTime.Now;
                                        double endDistance = distance;
                                        // Calculates and displays results
                                        double speed = (startDistance - endDistance) / ((double)(endTime - startTime).TotalSeconds);
                                        lblTime.Content = "Speed: " + speed;
                                        lblTime.Content = "Distance traveled: " + Math.Round((startDistance - endDistance), 2) + " (m)" + "\nTime: "
                                                + Math.Round(((double)(endTime - startTime).TotalSeconds), 2) + " (s)" + "\nSpeed: " + Math.Round(speed, 2) + " (m/s)";
                                    }
                                }
                                #endregion
                                #region 2-BALANCE
                                // Test #2: Balance
                                if (dispatcherTimer.IsEnabled && curTest == 2)
                                {
                                    // Ankles used to detect when a step is taken
                                    Joint leftAnkle = body.Joints[JointType.AnkleLeft];
                                    Joint rightAnkle = body.Joints[JointType.AnkleRight];
                                    // Record initial position of feet
                                    if (!balanceStart)
                                    {
                                        balanceStart = true;
                                        blx = leftAnkle.Position.X;
                                        bly = leftAnkle.Position.Y;
                                        blz = leftAnkle.Position.Z;
                                        brx = rightAnkle.Position.X;
                                        bry = rightAnkle.Position.Y;
                                        brz = rightAnkle.Position.Z;
                                    }
                                    // After initial position of feet recorded, detect if a step is taken
                                    else
                                    {
                                        // Calculate distance of current ankle position to initial ankle position
                                        Vector3D a1 = new Vector3D(leftAnkle.Position.X - blx, leftAnkle.Position.Y - bly, leftAnkle.Position.Z - blz);
                                        Vector3D a2 = new Vector3D(rightAnkle.Position.X - brx, rightAnkle.Position.Y - bry, rightAnkle.Position.Z - brz);
                                        // If either ankle moves 1 decimeter, test failed
                                        if (a1.Length > BA_STEP || a2.Length > BA_STEP)
                                        {
                                            dispatcherTimer.Stop();
                                            lblTime.Content = "FAILED";
                                        }
                                        // If 30 seconds passes without a fail, test passed
                                        else if ((DateTime.Now - startTime).Seconds >= BA_TIME)
                                        {
                                            dispatcherTimer.Stop();
                                            lblTime.Content = "PASSED";
                                        }
                                    }
                                }
                                #endregion
                                #region 3-GET_UP_AND_GO
                                // Test #3: Get-up-and-go
                                if (dispatcherTimer.IsEnabled && curTest == 3)
                                {
                                    // Record initial position of mid-spine
                                    if (!startedTracking)
                                    {
                                        yi = spineMid.Position.Y;
                                        startedTracking = true;
                                    }
                                    // If subject begins to stand (mid-spine rises 5 cm), begin test
                                    if (spineMid.Position.Y - yi >= GUAG_HEIGHT && !hasStood)
                                    {
                                        startTime = DateTime.Now;
                                        lblTime.Visibility = Visibility.Visible;
                                        hasStood = true;
                                    }
                                    // Mark when subject leaves Kinect's range
                                    if (!hasLeft && distance >= GUAG_LEAVE)
                                    {
                                        hasLeft = true;
                                    }
                                    // Mark when subject is returning (important for end of test), will occur when
                                    // tracked between 3 and 3.5 meters (range chosen to eliminate error)
                                    if (hasLeft && !comingBack && distance <= GUAG_RETURN_MAX && distance >= GUAG_RETURN_MIN)
                                    {
                                        comingBack = true;
                                    }
                                    // Stops test when subject is within 2.5 meters of Kinect, and is sitting down
                                    if (comingBack && distance <= GUAG_SIT_DIST && spineMid.Position.Y - yi <= GUAG_HEIGHT)
                                    {
                                        dispatcherTimer.Stop();
                                        lblTime.Content = "Time/Score: " + Math.Round((DateTime.Now - startTime).TotalSeconds, 3) + " (s)";
                                    }
                                }
                                #endregion
                                #region 4-CHAIR_RISE
                                // Test #4: Chair-rise
                                if (dispatcherTimer.IsEnabled && curTest == 4)
                                {
                                    // Record initial position of mid-spine, test has not started yet
                                    if (!chairStart)
                                    {
                                        startedTracking = false; // Set true initially
                                        yi = spineMid.Position.Y;
                                        chairStart = true;
                                    }
                                    // If button has been pressed, and test will begin soon/has begun
                                    else
                                    {
                                        // Start test when subject begins to stand (mid-spine rises 5 cm)
                                        if (!startedTracking && (spineMid.Position.Y - yi) >= CR_START_HEIGHT)
                                        {
                                            startedTracking = true;
                                            startTime = DateTime.Now;
                                        }
                                        // Has not stood up if still 1 decimeter from starting height
                                        else if (Math.Abs(spineMid.Position.Y - yi) <= CR_HEIGHT)
                                        {
                                            hasStood = false;
                                        }
                                        // Counts one chair-rise, when subject rises by 1 decimeter
                                        if (Math.Abs(spineMid.Position.Y - yi) >= CR_HEIGHT && !hasStood)
                                        {
                                            numUp++;
                                            lblTime.Content = "Time/Score: " + numUp;
                                            hasStood = true;
                                        }
                                        // Stops test when 5 chair-rises have been completed, and subject is sitting down
                                        if (numUp >= CR_REPS) // && spineMid.Position.Y - yi <= CR_START_HEIGHT)
                                        {
                                            dispatcherTimer.Stop();
                                            lblTime.Content = "Time/Score: " + Math.Round((DateTime.Now - startTime).TotalSeconds, 3) + " (s)";
                                        }
                                    }
                                }
                                #endregion
                            }
                        }
                    }
                }
            }
            #endregion
        }
        #endregion

        #region OUR_METHODS
        /*
         * Countdown method used to count down the balance test
         */
        private void countdown_Tick(object sender, EventArgs e)
        {
            // First four if statements count down from 3 to begin
            if (lblTime.Content.Equals(""))
                lblTime.Content = "3";
            else if (lblTime.Content.Equals("3"))
                lblTime.Content = "2";
            else if (lblTime.Content.Equals("2"))
                lblTime.Content = "1";
            else if (lblTime.Content.Equals("1"))
                lblTime.Content = "Begin!";
            // After begin, following code is executed
            else if (lblTime.Content.Equals("Begin!"))
            {
                dispatcherTimer.Start(); // Starts stopwatch
                startTime = DateTime.Now; // Records start time of balance test (to have an accurate stopwatch
                countdown.Stop(); // Stops running the timer, last segment of code has been reached
            }
        }
        /*
         * Shows the Kinect's skeletal tracker - useful in seeing if a body is being tracked
         */
        private void Body_Click(object sender, RoutedEventArgs e)
        {
            _displayBody = !_displayBody; // Displays skeletal figure for tracked bodies
        }
        /*
         * Starts the balance test
         */
        private void Balance_Click(object sender, RoutedEventArgs e)
        {
            countdown = new DispatcherTimer(); // Initializes countdown timer
            countdown.Tick += countdown_Tick; // Tells which method will make the timer tick?
            countdown.Interval = new TimeSpan(0, 0, 1); // Sets interval of timer to be 1 second
            lblTime.Content = "";
            curTest = 2; // Balance is test #2
            balanceStart = false; // Does not track initial position of feet yet
            countdown.Start(); // Starts countdown
        }
        /*
         * Starts the gait-speed test
         */
        private void Gait_Speed_Click(object sender, RoutedEventArgs e)
        {
            curTest = 1; // Gait-speed is test #1
            lblTime.Content = "Time/Score:";
            startedTracking = false; // Subject hasn't started walking
            dispatcherTimer.Start(); // Starts stopwatch
            startTime = DateTime.Now; // Initializes start time for stopwatch
        }
        /*
         * Starts the get-up-and-go test
         */
        private void Get_Up_and_Go_Click_1(object sender, RoutedEventArgs e)
        {
            lblTime.Content = "Time/Score:";
            curTest = 3; // Get-up-and-go is test #3
            startedTracking = false;
            hasStood = false; // Subject has not stood up yet
            hasLeft = false; // Subject has not left the screen yet
            comingBack = false; // Subject is not returning to the screen yet
            dispatcherTimer.Start();
            startTime = DateTime.Now; // Initialize start time
            lblTime.Visibility = Visibility.Hidden; // Hide stopwatch until test starts
        }
        /*
         * Starts the chair-rise test
         */
        private void Chair_Rise_Click_1(object sender, RoutedEventArgs e)
        {
            lblTime.Content = "Time/Score:";
            curTest = 4; // Chair-rise is test #4
            numUp = 0; // Initialize number of "ups" to 0
            chairStart = false; // Subject has not started the test yet
            startedTracking = true; // Safeguard to prevent test from terminating early
            startTime = DateTime.Now;
            dispatcherTimer.Start();
        }
        #endregion
    }

    #region MODE
    /*
     * Defines each camera mode available to be displayed. In this version, we
     * only use the Color mode.
     */
    public enum Mode
    {
        Color,
        Depth,
        Infrared
    }
    #endregion
}
