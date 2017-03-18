using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Drawing;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Microsoft.Kinect.VisualGestureBuilder;
using Microsoft.Kinect.Face;
using Microsoft.Kinect.Fusion;
using System.Windows.Media.Imaging;
using System.Speech.Synthesis;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;

namespace Robonic
{
    namespace Kinect
    {
        public delegate void RobonicEventHandler(EventArgs e);
        //Completed
        public class KinectIntraction
        {
            public class SensorAvailibilityEventArgs : EventArgs
            {
                public bool Available;
                public SensorAvailibilityEventArgs(bool available)
                {
                    Available = available;
                }
            }

            protected static KinectSensor Sensor; public event RobonicEventHandler SensorNotAvailable; public event RobonicEventHandler SensorAvailable;
            public bool Connect(int timeout = 0)
            {
                Sensor = KinectSensor.GetDefault();
                Sensor.IsAvailableChanged += Sensor_IsAvailableChanged;
                try
                {
                    if (!Sensor.IsOpen)
                    {
                        try
                        {
                            Sensor.Open();
                        }
                        catch (Exception)
                        {
                            throw new RobonikException("Unable to open kinect.");
                        }
                    }
                }
                catch (Exception) { }

                if (timeout > 0)
                {
                    System.Threading.Thread.Sleep(timeout);
                }
                if (Sensor.IsOpen && Sensor.IsAvailable)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            public bool Disconnect()
            {
                if (Sensor.IsOpen)
                {
                    try
                    {
                        Sensor.Close();
                        if (!Sensor.IsOpen)
                        {
                            return true;
                        }
                    }
                    catch (Exception)
                    {
                        throw new RobonikException("Unable to close kinect");
                    }
                }
                return false;
            }

            private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
            {
                if (!e.IsAvailable)
                {
                    if (SensorNotAvailable != null)
                    {
                        SensorNotAvailable.Invoke(new SensorAvailibilityEventArgs(false));
                    }
                }
                else
                {
                    if (SensorAvailable != null)
                    {
                        SensorAvailable.Invoke(new SensorAvailibilityEventArgs(true));
                    }
                }
            }

            public bool SensorAvailibility
            {
                get
                {
                    return Sensor.IsAvailable;
                }
            }
        }
            [Serializable]
            public class RobonikException : Exception
            {   
                public RobonikException() { }
                public RobonikException(string message) : base(message) { }
            }
        //To Test
        public class GestureRecognition : KinectIntraction
        {
            VisualGestureBuilderFrameSource frame_source; VisualGestureBuilderFrameReader frame_reader;List<Tuple<string, bool, float>> discrete_stored = null;
            List<DiscreteGestureProperties> discrete_gestures = null; List<ContinuousGestureProperties> continuous_gestures = null;List<string> rec_report;
            List<string> unrecreported = null; List<Tuple<string, float>> continuous_stored = null;
            public event RobonicEventHandler GestureRecognizedEvent;bool have_discrete = false, have_continuous = false,datastoration = false;
            float global_confidence;public event RobonicEventHandler TrackingIdLostEvent; public event RobonicEventHandler GestureUnrecognizedEvent;

            public GestureRecognition(int trackingid, string database_address,List<ContinuousGestureProperties> continuous, List<DiscreteGestureProperties> discrete, float confidence = (float)0.7)
            {
                rec_report = new List<string>();
                unrecreported = new List<string>();
                discrete_gestures = discrete;
                continuous_gestures = continuous;
                global_confidence = confidence;
                frame_source = new VisualGestureBuilderFrameSource(Sensor, (ulong)trackingid);
                frame_source.TrackingIdLost += Frame_source_TrackingIdLost;
                if (discrete == null)
                {
                    if (continuous == null)
                    {
                        add_gestures(database_address, true,true);
                    }
                    else
                    {
                        add_gestures(database_address, true, false);
                        add_some_continuous_gestures(database_address);
                    }
                }
                else
                {
                    if (continuous == null)
                    {
                        add_gestures(database_address, false, true);
                        add_some_discrete_gestures(database_address);
                    }
                    else
                    {
                        add_some_discrete_gestures(database_address);
                        add_some_continuous_gestures(database_address);
                    }
                }
                frame_reader = frame_source.OpenReader();
                if (frame_reader != null)
                {
                    frame_reader.IsPaused = true;
                    frame_reader.FrameArrived += Reader_source_FrameArrived;
                }
            }
            public class GestureRecognizedEventArgs : EventArgs
            {
                public float Value;public string GestureName;public GestureType Type;
                public GestureRecognizedEventArgs(float value,string gestureName, GestureType type)
                {
                    Type = type;
                    Value = value;
                    GestureName = gestureName;
                }
            }
            public class GestureUnrecognizedEventArgs : EventArgs
            {
                public string Name;float Confidence;
                public GestureUnrecognizedEventArgs(string name, float confidence)
                {
                    Name = name;
                    Confidence = confidence;
                }
            }
            public class TrackingIdLostEventArgs : EventArgs
            {
                public int LostID;
                public TrackingIdLostEventArgs(int trackingid)
                {
                    LostID = trackingid;
                }
            }
            public struct DiscreteGestureProperties
            {
                public string GestureName;
                public float ConfidenceRatio;
                public const float AnyConfidence = -1;
                public DiscreteGestureProperties(string name, float confidence) { GestureName = name; ConfidenceRatio = confidence;}
            }
            public struct ContinuousGestureProperties
            {
                public string GestureName;
                public Tuple<float,float> Progress;
                public const float AnyConfidence = -1;
                public ContinuousGestureProperties(string name, Tuple<float, float> progress) { GestureName = name; Progress = progress; }
            }
            public void RestartProgressReporting(string continuous_name)
            {
                if (rec_report.Contains(continuous_name))
                {
                    rec_report.Remove(continuous_name);
                }
            }
            public void Detach()
            {
                Pause = true;
                if (frame_reader != null)
                {
                    frame_reader.FrameArrived -= Reader_source_FrameArrived;
                    frame_reader.Dispose();
                    frame_reader = null;
                }

                if (frame_source != null)
                {
                    frame_source.TrackingIdLost -= Frame_source_TrackingIdLost;
                    frame_source.Dispose();
                    frame_source = null;
                }
            }
            public bool DataStoration
            {
                get
                {
                    return datastoration;
                }
                set
                {
                    datastoration = value;
                }
            }
            void add_gestures(string address, bool discrets, bool continuous)
            {
                using (VisualGestureBuilderDatabase database = new VisualGestureBuilderDatabase(address))
                {
                    foreach (Gesture gesture in database.AvailableGestures)
                    {
                        if (gesture.GestureType == GestureType.Continuous && !continuous)
                        {
                            continue;
                        }
                        else
                        {
                            if (gesture.GestureType == GestureType.Discrete && !discrets)
                            {
                                continue;
                            }
                        }
                        frame_source.AddGesture(gesture);
                        if (gesture.GestureType == GestureType.Continuous)
                        {
                            have_continuous = true;
                        }
                        else
                        {
                            have_discrete = true;
                        }
                    }
                }
            }
            public bool IsDiscreteDetected(string name)
            {
                if (DataStoration)
                {
                    foreach (var item in discrete_stored)
                    {
                        if (item.Item1 == name)
                        {
                            return item.Item2;
                        }
                    }
                    throw new RobonikException("Gesture name not found.");
                }
                else
                {
                    throw new RobonikException("Gesture data storation is off.");
                }
            }
            public float DiscreteConfidence(string name)
            {
                if (DataStoration)
                {
                    foreach (var item in discrete_stored)
                    {
                        if (item.Item1 == name)
                        {
                            return item.Item3;
                        }
                    }
                    throw new RobonikException("Gesture name not found.");
                }
                else
                {
                    throw new RobonikException("Gesture data storation is off.");
                }
            }
            public float ContinuousProgress(string name)
            {
                if (DataStoration)
                {
                    foreach (var item in continuous_stored)
                    {
                        if (item.Item1 == name)
                        {
                            return item.Item2;
                        }
                    }
                    throw new RobonikException("Gesture name not found.");
                }
                else
                {
                    throw new RobonikException("Gesture data storation is off.");
                }
            }
            void add_some_discrete_gestures(string address)
            {
                using (VisualGestureBuilderDatabase database = new VisualGestureBuilderDatabase(address))
                {
                    foreach (Gesture gesture in database.AvailableGestures)
                    {
                        foreach (var item in discrete_gestures)
                        {
                            if (gesture.Name == item.GestureName)
                            {
                                frame_source.AddGesture(gesture);
                                have_discrete = true;
                            }
                        }
                    }
                }
            }
            void add_some_continuous_gestures(string address)
            {
                using (VisualGestureBuilderDatabase database = new VisualGestureBuilderDatabase(address))
                {
                    foreach (Gesture gesture in database.AvailableGestures)
                    {
                        foreach (var item in continuous_gestures)
                        {
                            if (gesture.Name == item.GestureName)
                            {
                                frame_source.AddGesture(gesture);
                                have_continuous = true;
                            }
                        }
                    }
                }
            }
            private void Reader_source_FrameArrived(object sender, VisualGestureBuilderFrameArrivedEventArgs e)
            {
                using (VisualGestureBuilderFrame frame = e.FrameReference.AcquireFrame())
                {
                    if (frame != null)
                    {
                        if (have_continuous)
                        {
                            if (datastoration || GestureRecognizedEvent != null)
                            {
                                IReadOnlyDictionary<Gesture, ContinuousGestureResult> continuousResults = frame.ContinuousGestureResults;
                                if (continuousResults != null)
                                {
                                    if (continuous_gestures == null)
                                    {
                                        continuous_stored = new List<Tuple<string, float>>();
                                        foreach (Gesture gesture in frame_source.Gestures)
                                        {
                                            if (gesture.GestureType == GestureType.Continuous)
                                            {
                                                ContinuousGestureResult result;
                                                continuousResults.TryGetValue(gesture, out result);
                                                if (result != null)
                                                {
                                                    if (datastoration)
                                                    {
                                                        continuous_stored.Add(new Tuple<string, float>(gesture.Name, result.Progress));
                                                    }
                                                    if (GestureRecognizedEvent != null)
                                                    {
                                                        if (!rec_report.Contains(gesture.Name))
                                                        {
                                                            GestureRecognizedEvent.Invoke(new GestureRecognizedEventArgs(result.Progress, gesture.Name, GestureType.Continuous));
                                                        }
                                                        else
                                                        {
                                                            rec_report.Add(gesture.Name);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        continuous_stored = new List<Tuple<string, float>>();
                                        foreach (Gesture gesture in frame_source.Gestures)
                                        {
                                            if (gesture.GestureType == GestureType.Continuous)
                                            {
                                                foreach (var g in continuous_gestures)
                                                {
                                                    if (g.GestureName == gesture.Name)
                                                    {
                                                        ContinuousGestureResult result;
                                                        continuousResults.TryGetValue(gesture, out result);
                                                        if (result != null)
                                                        {
                                                            if (datastoration)
                                                            {
                                                                continuous_stored.Add(new Tuple<string, float>(gesture.Name, result.Progress));
                                                            }
                                                            if (result.Progress <= g.Progress.Item2 && result.Progress >= g.Progress.Item1 && GestureRecognizedEvent != null)
                                                            {
                                                                if (GestureRecognizedEvent != null)
                                                                {
                                                                    if (!rec_report.Contains(gesture.Name))
                                                                    {
                                                                        GestureRecognizedEvent.Invoke(new GestureRecognizedEventArgs(result.Progress, gesture.Name, GestureType.Continuous));
                                                                    }
                                                                    else
                                                                    {
                                                                        rec_report.Add(gesture.Name);
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if (have_discrete)
                        {
                            if (GestureRecognizedEvent != null || GestureRecognizedEvent != null || datastoration)
                            {
                                IReadOnlyDictionary<Gesture, DiscreteGestureResult> continuousResults = frame.DiscreteGestureResults;
                                if (continuousResults != null)
                                {
                                    if (discrete_gestures == null)
                                    {
                                        discrete_stored = new List<Tuple<string, bool, float>>();
                                        foreach (var gesture in frame_source.Gestures)
                                        {
                                            if (gesture.GestureType == GestureType.Discrete)
                                            {
                                                DiscreteGestureResult result;
                                                continuousResults.TryGetValue(gesture, out result);
                                                if (result != null)
                                                {
                                                    if (datastoration)
                                                    {
                                                        discrete_stored.Add(new Tuple<string, bool, float>(gesture.Name, result.Detected, result.Confidence));
                                                    }
                                                    if (GestureRecognizedEvent != null)
                                                    {
                                                        if (result.Detected)
                                                        {
                                                            if (result.Confidence >= global_confidence)
                                                            {
                                                                if (GestureRecognizedEvent != null && !rec_report.Contains(gesture.Name))
                                                                {
                                                                    GestureRecognizedEvent.Invoke(new GestureRecognizedEventArgs(result.Confidence, gesture.Name, GestureType.Discrete));
                                                                    rec_report.Add(gesture.Name);
                                                                    if (unrecreported.Contains(gesture.Name))
                                                                    {
                                                                        unrecreported.Remove(gesture.Name);
                                                                    }
                                                                }
                                                            }
                                                            else
                                                            {
                                                                if (rec_report.Contains(gesture.Name) && GestureUnrecognizedEvent != null)
                                                                {
                                                                    rec_report.Remove(gesture.Name);
                                                                    if (!unrecreported.Contains(gesture.Name))
                                                                    {
                                                                        GestureUnrecognizedEvent.Invoke(new GestureUnrecognizedEventArgs(gesture.Name, result.Confidence));
                                                                        unrecreported.Add(gesture.Name);
                                                                    }
                                                                }
                                                            }
                                                        }
                                                        else
                                                        {
                                                            if (rec_report.Contains(gesture.Name) && GestureUnrecognizedEvent != null)
                                                            {
                                                                GestureUnrecognizedEvent.Invoke(new GestureUnrecognizedEventArgs(gesture.Name, 0));
                                                                rec_report.Remove(gesture.Name);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        discrete_stored = new List<Tuple<string, bool, float>>();
                                        foreach (var gesture in frame_source.Gestures)
                                        {
                                            if (gesture.GestureType == GestureType.Discrete)
                                            {
                                                foreach (var g in discrete_gestures)
                                                {
                                                    if (g.GestureName == gesture.Name)
                                                    {
                                                        DiscreteGestureResult result;
                                                        continuousResults.TryGetValue(gesture, out result);
                                                        if (result != null)
                                                        {
                                                            if (datastoration)
                                                            {
                                                                discrete_stored.Add(new Tuple<string, bool, float>(gesture.Name, result.Detected, result.Confidence));
                                                            }
                                                            if (result.Detected)
                                                            {
                                                                if (result.Confidence >= g.ConfidenceRatio)
                                                                {
                                                                    if (GestureRecognizedEvent != null && !rec_report.Contains(gesture.Name))
                                                                    {
                                                                        GestureRecognizedEvent.Invoke(new GestureRecognizedEventArgs(result.Confidence, gesture.Name, GestureType.Discrete));
                                                                        rec_report.Add(gesture.Name);
                                                                        if (unrecreported.Contains(gesture.Name))
                                                                        {
                                                                            unrecreported.Remove(gesture.Name);
                                                                        }
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    if (rec_report.Contains(gesture.Name) && GestureUnrecognizedEvent != null)
                                                                    {
                                                                        rec_report.Remove(gesture.Name);
                                                                        if (!unrecreported.Contains(gesture.Name))
                                                                        {
                                                                            GestureUnrecognizedEvent.Invoke(new GestureUnrecognizedEventArgs(gesture.Name, result.Confidence));
                                                                            unrecreported.Add(gesture.Name);
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                            else
                                                            {
                                                                if (rec_report.Contains(gesture.Name) && GestureUnrecognizedEvent != null)
                                                                {
                                                                    GestureUnrecognizedEvent.Invoke(new GestureUnrecognizedEventArgs(gesture.Name, 0));
                                                                    rec_report.Remove(gesture.Name);
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }            
            }
            private void Frame_source_TrackingIdLost(object sender, Microsoft.Kinect.VisualGestureBuilder.TrackingIdLostEventArgs e)
            {
                if (TrackingIdLostEvent != null)
                {
                    TrackingIdLostEvent.Invoke(new TrackingIdLostEventArgs((int)e.TrackingId));
                }
            }
            public bool Pause
            {
                get
                {
                    if (frame_reader != null)
                    {
                        return frame_reader.IsPaused;
                    }
                    return false;
                }
                set
                {
                    if (frame_reader != null)
                    {
                        frame_reader.IsPaused = value;
                    }
                }
            }
        }
        //To Test
        public class ColorStream : KinectIntraction
        {
            ColorFrameReader frame_reader;int frame_width, frame_hight;bool datastoration = false;System.Drawing.Bitmap data;
            public event RobonicEventHandler FrameReceivedEvent;
            public ColorStream()
            {
                if (Sensor.IsAvailable)
                {
                    FrameDescription frame_describtion = Sensor.ColorFrameSource.FrameDescription;
                    frame_width = frame_describtion.Width;
                    frame_hight = frame_describtion.Height;
                    frame_reader = Sensor.ColorFrameSource.OpenReader();
                    frame_reader.FrameArrived += Frame_reader_FrameArrived;
                }
                else
                {
                    throw new RobonikException("Sensor is not available.");
                }
            }
            private void Frame_reader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
            {
                using (ColorFrame frame = e.FrameReference.AcquireFrame())
                {
                    if (frame != null && (datastoration || FrameReceivedEvent != null))
                    {
                        byte[] imagedata = null; frame.CopyConvertedFrameDataToArray(imagedata, ColorImageFormat.Bgra);
                        using (var ms = new System.IO.MemoryStream(imagedata))
                        {
                            data = new System.Drawing.Bitmap(ms);
                        }
                        if (FrameReceivedEvent != null)
                        {
                            FrameReceivedEvent.Invoke(new ColorFrameReceivedEventArgs(data));
                        }
                    }
                }
            }
            public class ColorFrameReceivedEventArgs : EventArgs
            {
                System.Drawing.Bitmap FrameData;
                public ColorFrameReceivedEventArgs(System.Drawing.Bitmap data)
                {
                    FrameData = data;
                }
            }
            public System.Drawing.Bitmap GetData()
            {
                if (datastoration)
                {
                    return data;
                }
                else
                {
                    throw new RobonikException("Data storation is off.");
                }
            }
            public int FrameWidth
            {
                get
                {
                    return frame_width;
                }
            }
            public int FrameHight
            {
                get
                {
                    return frame_hight;
                }
            }
            public bool DataStoration
            {
                get
                {
                    return datastoration;
                }
                set
                {
                    datastoration = value;
                }
            }
            public bool Pause
            {
                get
                {
                    if (frame_reader != null)
                    {
                        return frame_reader.IsPaused;
                    }
                    return false;
                }
                set
                {
                    if (frame_reader != null)
                    {
                        frame_reader.IsPaused = value;
                    }
                }
            }
        }
        //InComplete
        //public class InfraredStream : KinectIntraction
        //{
        //    InfraredFrameReader frame_reader; int frame_width, frame_hight; bool datastoration = false; IntPtr data;int size;int bytes_per_pixel;
        //    public event RobonicEventHandler FrameReceivedEvent;
        //    public InfraredStream()
        //    {
        //        if (Sensor.IsAvailable)
        //        {
        //            FrameDescription frame_describtion = Sensor.ColorFrameSource.FrameDescription;
        //            bytes_per_pixel = (int)frame_describtion.BytesPerPixel;
        //            frame_width = frame_describtion.Width;
        //            frame_hight = frame_describtion.Height;
        //            frame_reader = Sensor.InfraredFrameSource.OpenReader();
        //            frame_reader.FrameArrived += Frame_reader_FrameArrived;
        //        }
        //        else
        //        {
        //            throw new RobonikException("Sensor is not available.");
        //        }
        //    }
        //    private void Frame_reader_FrameArrived(object sender, InfraredFrameArrivedEventArgs e)
        //    {
        //        using (InfraredFrame frame = e.FrameReference.AcquireFrame())
        //        {
        //            if (frame != null && (datastoration || FrameReceivedEvent != null))
        //            {
        //                using (KinectBuffer buffer = frame.LockImageBuffer())
        //                {
        //                    frame.CopyFrameDataToIntPtr(data,buffer.Size);
        //                    size = (int)buffer.Size;
        //                }
        //            }
        //        }
        //    }
        //    unsafe void dataToBitmap()
        //    {
        //        ushort* frameData = (ushort*)data;
        //        (byte[]) data 
        //        // get the pointer to the bitmap's back buffer
        //        float* backBuffer = (float*)this.infraredBitmap.BackBuffer;
        //        byte[] byte_array = new byte[FrameHight * FrameWidth * bytes_per_pixel];
        //        // process the infrared data
        //        for (int i = 0; i < (size / bytes_per_pixel); ++i)
        //        {
        //            // since we are displaying the image as a normalized grey scale image, we need to convert from
        //            // the ushort data (as provided by the InfraredFrame) to a value from [InfraredOutputValueMinimum, InfraredOutputValueMaximum]
        //            byte_array[i] = Math.Min(1.0F, (((float)byte_array[i] / (float)ushort.MaxValue * 0.75F) * (1.0f - 0.1F)) + 0.1F);
        //        }
        //        if (FrameReceivedEvent != null)
        //        {
        //            FrameReceivedEvent.Invoke(new InfraredFrameReceivedEventArgs(data));
        //        }
        //    }
        //    public void Capture(string path = "")
        //    {
        //        if (path != "")
        //        {BitConverter.GetBytes(data)
        //            BitmapToPngEncoder.CreatePng(ref)
        //        }
        //    }
        //    public class InfraredFrameReceivedEventArgs : EventArgs
        //    {
        //        ushort[] FrameData;
        //        public InfraredFrameReceivedEventArgs(ushort[] data)
        //        {
        //            FrameData = data;
        //        }
        //    }
        //    public ushort[] GetData()
        //    {
        //        if (datastoration)
        //        {
        //            return data;
        //        }
        //        else
        //        {
        //            throw new RobonikException("Data storation is off.");
        //        }
        //    }
        //    public int FrameWidth
        //    {
        //        get
        //        {
        //            return frame_width;
        //        }
        //    }
        //    public int FrameHight
        //    {
        //        get
        //        {
        //            return frame_hight;
        //        }
        //    }
        //    public bool DataStoration
        //    {
        //        get
        //        {
        //            return datastoration;
        //        }
        //        set
        //        {
        //            datastoration = value;
        //        }
        //    }
        //    public bool Pause
        //    {
        //        get
        //        {
        //            if (frame_reader != null)
        //            {
        //                return frame_reader.IsPaused;
        //            }
        //            return false;
        //        }
        //        set
        //        {
        //            if (frame_reader != null)
        //            {
        //                frame_reader.IsPaused = value;
        //            }
        //        }
        //    }
        //}
        static class BitmapToPngEncoder
        {
            public enum PictureSource
            {
                Depth = 0
                , Color = 1
                , Infrared = 2
                , LongExposureInfrared = 3
            }
            public static void CreatePng(ref Bitmap bitmap, PictureSource picture_source, string folder_path = @"C:\Robonic\KinectCapture")
            {
                if (!Directory.Exists(folder_path))
                {
                    Directory.CreateDirectory(folder_path);
                }
                DateTime Now = DateTime.UtcNow;
                folder_path = folder_path + "\\" + picture_source.ToString()+Now.Ticks.ToString();
                BitmapEncoder encoder = new PngBitmapEncoder();
                try
                {
                    using (FileStream fs = new FileStream(folder_path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }
                }
                catch (IOException)
                {
                    throw new RobonikException("Unable to save image.");
                }
            }
        }
        //Completed
        public class LongExposureInfraredStream : KinectIntraction
        {
            LongExposureInfraredFrameReader frame_reader; int frame_width, frame_hight; bool datastoration = false; ushort[] data;
            public event RobonicEventHandler FrameReceivedEvent;
            public LongExposureInfraredStream()
            {
                if (Sensor.IsAvailable)
                {
                    FrameDescription frame_describtion = Sensor.ColorFrameSource.FrameDescription;
                    frame_width = frame_describtion.Width;
                    frame_hight = frame_describtion.Height;
                    frame_reader = Sensor.LongExposureInfraredFrameSource.OpenReader();
                    frame_reader.FrameArrived += Frame_reader_FrameArrived;
                }
                else
                {
                    throw new RobonikException("Sensor is not available.");
                }
            }

            private void Frame_reader_FrameArrived(object sender, LongExposureInfraredFrameArrivedEventArgs e)
            {
                using (LongExposureInfraredFrame frame = e.FrameReference.AcquireFrame())
                {
                    if (frame != null && (datastoration || FrameReceivedEvent != null))
                    {
                        ushort[] imagedata = null; frame.CopyFrameDataToArray(imagedata);
                        if (FrameReceivedEvent != null)
                        {
                            FrameReceivedEvent.Invoke(new InfraredFrameReceivedEventArgs(data));
                        }
                    }
                }
            }
            public class InfraredFrameReceivedEventArgs : EventArgs
            {
                ushort[] FrameData;
                public InfraredFrameReceivedEventArgs(ushort[] data)
                {
                    FrameData = data;
                }
            }
            public ushort[] GetData()
            {
                if (datastoration)
                {
                    return data;
                }
                else
                {
                    throw new RobonikException("Data storation is off.");
                }
            }
            public int FrameWidth
            {
                get
                {
                    return frame_width;
                }
            }
            public int FrameHight
            {
                get
                {
                    return frame_hight;
                }
            }
            public bool DataStoration
            {
                get
                {
                    return datastoration;
                }
                set
                {
                    datastoration = value;
                }
            }
            public bool Pause
            {
                get
                {
                    if (frame_reader != null)
                    {
                        return frame_reader.IsPaused;
                    }
                    return false;
                }
                set
                {
                    if (frame_reader != null)
                    {
                        frame_reader.IsPaused = value;
                    }
                }
            }
        }
        //To Test
        public class BodyData : KinectIntraction
        {
            BodyFrameReader frame_reader; Body[] bodies; bool datastoration = false, calc_bone_id = false; public event RobonicEventHandler BodiesDataReceivedEvent;
            List<BodyProperties> properties_list = null; List<Tuple<int, string, double>> bone_id = null; int body_count = 0;bool illegaldrop = false;
            List<JointSpeed> joint_speed_data;bool accurate_hand = false;HandState last_right = HandState.Closed, last_left = HandState.Closed;
            int righthand_frame_count = 0, lefthand_frame_count = 0, righthand_untrack_frame_count = 0, lefthand_untrack_frame_count = 0;

            const double bonelength_telorance = 0.1;const int hand_frame_wait = 3;const int handuntrack_frame_wait = 5;
            const float hand_speed_limit = (float)0.5, leg_speed_limit = (float)0.3, neck_speed_limit = (float)0.3;

            public BodyData()
            {
                if (Sensor.IsAvailable)
                {
                    bone_id = new List<Tuple<int, string, double>>();
                    joint_speed_data = new List<JointSpeed>();
                    frame_reader = Sensor.BodyFrameSource.OpenReader();
                    frame_reader.IsPaused = false;
                    frame_reader.FrameArrived += Frame_reader_FrameArrived;
                }
                else
                {
                    throw new RobonikException("Sensor is not avilable.");
                }
            }
            public struct BodyProperties
            {
                public IReadOnlyDictionary<JointType, Joint> JointsLocationData;
                public IReadOnlyDictionary<JointType, JointOrientation> JointsOrganizationData;
                public int TrackingId;
                public bool IsTracked, IsRestricted,IsLegalHead, IsLegalLegRight, IsLegalLegLeft, IsLegalArmRight, IsLegalArmLeft;
                public TrackingState LeanTrackingState;
                public TrackingConfidence RightHandConfidence, LeftHandConfidence;
                public float XLean, YLean;
                public HandState RightHandState, LeftHandState;
                public BodyProperties(IReadOnlyDictionary<JointType, Joint> joint_location_data, IReadOnlyDictionary<JointType, JointOrientation> joint_organization_data, bool is_tracked, bool is_restricted, TrackingState lean_tacking_state, float x_lean, float y_lean, HandState right_hand_state, HandState left_hand_state, TrackingConfidence right_hand_confidence, TrackingConfidence left_hand_confidence,bool is_legal_head,bool is_legal_leg_right,bool is_legal_leg_left, bool is_legal_arm_right,bool is_legal_arm_left, int tracking_id)
                {
                    LeanTrackingState = lean_tacking_state;
                    JointsLocationData = joint_location_data;
                    TrackingId = tracking_id;
                    IsRestricted = is_restricted;
                    IsTracked = is_tracked;
                    JointsOrganizationData = joint_organization_data;
                    XLean = x_lean;
                    YLean = y_lean;
                    RightHandConfidence = right_hand_confidence;
                    LeftHandConfidence = left_hand_confidence;
                    RightHandState = right_hand_state;
                    LeftHandState = left_hand_state;
                    IsLegalHead = is_legal_head;
                    IsLegalArmRight = is_legal_arm_right;
                    IsLegalArmLeft = is_legal_arm_left;
                    IsLegalLegRight = is_legal_leg_right;
                    IsLegalLegLeft = is_legal_leg_left;
                }
            }
            public void AddToJointSpeedList(double bone, JointType joint)
            {
                foreach (var item in joint_speed_data)
                {
                    if (IsBoneMatch(bone, item.Bone) && item.Joint == joint)
                    {
                        return;
                    }
                }
                joint_speed_data.Add(new JointSpeed(joint, bone));
            }
            public struct JointSpeed
            {
                public JointType Joint; public double Bone; float dx, dy, dz; float last_x, last_y, last_z;
                TimeSpan last_time, dt; bool first_data;
                public JointSpeed(JointType joint, double bone)
                {
                    Joint = joint;
                    Bone = bone;
                    dx = 0;
                    dy = 0;
                    dz = 0;
                    last_time = TimeSpan.Zero;
                    dt = TimeSpan.Zero;
                    last_x = 0;
                    last_y = 0;
                    last_z = 0;
                    first_data = true;
                }
                public void Reset()
                {
                    this = new JointSpeed(Joint, Bone);
                }
                public void AddData(TimeSpan time, float x_position, float y_position, float z_position)
                {
                    if (first_data)
                    {
                        last_time = time;
                        last_x = x_position;
                        last_y = y_position;
                        last_z = z_position;
                        first_data = false;
                    }
                    else
                    {
                        dx += x_position - last_x;
                        dy += y_position - last_y;
                        dz += z_position - last_z;
                        dt += time - last_time;
                        last_x = x_position;
                        last_y = y_position;
                        last_z = z_position;
                        last_time = time;
                    }
                }
                public float XSpeed()
                {
                    return dx * 1000 / dt.Milliseconds;
                }
                public float YSpeed()
                {
                    return dy * 1000 / dt.Milliseconds;
                }
                public float ZSpeed()
                {
                    return dz * 1000 / dt.Milliseconds;
                }
            }
            private void Frame_reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
            {
                using (BodyFrame frame = e.FrameReference.AcquireFrame())
                {
                    if (frame != null)
                    {
                        if (BodiesDataReceivedEvent == null && !datastoration)
                        {
                            BodyCount = frame.BodyCount;
                        }
                        bodies = new Body[6];
                        if (BodiesDataReceivedEvent != null || datastoration)
                        {
                            int tracked_body_count = 0;
                            properties_list = new List<BodyProperties>();
                            frame.GetAndRefreshBodyData(bodies);
                            foreach (var body in bodies)
                            {
                                if (body.IsTracked)
                                {
                                    HandState rightstate, leftstate;
                                    tracked_body_count++;
                                    if (accurate_hand)
                                    {
                                        rightstate = body.HandRightState;leftstate = body.HandLeftState;
                                        if (rightstate == last_right)
                                        {
                                            if (rightstate == HandState.NotTracked)
                                            {
                                                if (righthand_untrack_frame_count == handuntrack_frame_wait)
                                                {
                                                    rightstate = HandState.NotTracked;
                                                }
                                                else
                                                {
                                                    righthand_untrack_frame_count++;
                                                    rightstate = last_right;
                                                }
                                            }
                                            else
                                            {
                                                if (last_right == HandState.NotTracked)
                                                {
                                                    last_right = rightstate;
                                                }
                                                else
                                                {
                                                    if (righthand_frame_count == hand_frame_wait)
                                                    {
                                                        if (rightstate == HandState.Lasso)
                                                        {
                                                            rightstate = HandState.Open;
                                                        }
                                                        if (rightstate == HandState.Unknown)
                                                        {
                                                            if (last_right == HandState.Open)
                                                            {
                                                                rightstate = HandState.Closed;
                                                            }
                                                            else
                                                            {
                                                                if (last_right == HandState.Closed)
                                                                {
                                                                    rightstate = HandState.Open;
                                                                }
                                                            }
                                                        }
                                                        last_right = rightstate;
                                                    }
                                                    else
                                                    {
                                                        righthand_frame_count++;
                                                        rightstate = last_right;
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        {
                                            righthand_frame_count = 0;
                                            righthand_untrack_frame_count = 0;
                                        }
                                        if (leftstate == last_left)
                                        {
                                            if (leftstate == HandState.NotTracked)
                                            {
                                                if (lefthand_untrack_frame_count == handuntrack_frame_wait)
                                                {
                                                    leftstate = HandState.NotTracked;
                                                }
                                                else
                                                {
                                                    lefthand_untrack_frame_count++;
                                                    leftstate = last_left;
                                                }
                                            }
                                            else
                                            {
                                                if (last_left == HandState.NotTracked)
                                                {
                                                    last_left = leftstate;
                                                }
                                                else
                                                {
                                                    if (lefthand_frame_count == hand_frame_wait)
                                                    {
                                                        if (leftstate == HandState.Lasso)
                                                        {
                                                            leftstate = HandState.Open;
                                                        }
                                                        if (leftstate == HandState.Unknown)
                                                        {
                                                            if (last_left == HandState.Open)
                                                            {
                                                                leftstate = HandState.Closed;
                                                            }
                                                            else
                                                            {
                                                                if (last_left == HandState.Closed)
                                                                {
                                                                    leftstate = HandState.Open;
                                                                }
                                                            }
                                                        }
                                                        last_left = leftstate;
                                                    }
                                                    else
                                                    {
                                                        lefthand_frame_count++;
                                                        leftstate = last_right;
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        {
                                            lefthand_frame_count = 0;
                                            lefthand_untrack_frame_count = 0;
                                        }
                                        properties_list.Add(new BodyProperties(body.Joints, body.JointOrientations, body.IsTracked, body.IsRestricted, body.LeanTrackingState, body.Lean.X, body.Lean.Y, rightstate, leftstate, body.HandRightConfidence, body.HandLeftConfidence, true,true,true,true,true, (int)body.TrackingId));
                                    }
                                    else
                                    {
                                        properties_list.Add(new BodyProperties(body.Joints, body.JointOrientations, body.IsTracked, body.IsRestricted, body.LeanTrackingState, body.Lean.X, body.Lean.Y, body.HandRightState, body.HandLeftState, body.HandRightConfidence, body.HandLeftConfidence, true, true, true,true,true,(int)body.TrackingId));
                                    }
                                }
                                else
                                {
                                    properties_list.Add(new BodyProperties(body.Joints, body.JointOrientations, false, false, TrackingState.NotTracked, 0, 0, HandState.NotTracked, HandState.NotTracked, 0, 0, true,true,true,true,true,(int)body.TrackingId));
                                }
                                for (int i = 0; i < joint_speed_data.Count; i++)
                                {
                                    if (IsTracked(GetTrackingIdByBoneLength(joint_speed_data[i].Bone)))
                                    {
                                        if (IsBoneMatch(joint_speed_data[i].Bone, GetBoneLengthByTrackingId((int)body.TrackingId)))
                                        {
                                            if (body.Joints[joint_speed_data[i].Joint].TrackingState != TrackingState.NotTracked)
                                            {
                                                joint_speed_data[i].AddData(frame.RelativeTime, body.Joints[joint_speed_data[i].Joint].Position.X, body.Joints[joint_speed_data[i].Joint].Position.Y, body.Joints[joint_speed_data[i].Joint].Position.Z);
                                            }
                                            else
                                            {
                                                joint_speed_data[i].Reset();
                                            }
                                        }
                                    }
                                    else
                                    {
                                        joint_speed_data[i].Reset();
                                    }
                                }
                            }
                            BodyCount = tracked_body_count;
                            if (illegaldrop)
                            {
                                bool foundn = false,foundar = false, foundal = false, foundlr = false, foundll = false;
                                foreach (var item in bone_id)
                                {
                                    foreach (var data in joint_speed_data)
                                    {
                                        if (IsBoneMatch(data.Bone,item.Item3))
                                        {
                                            if (data.Joint == JointType.Neck)
                                            {
                                                foundn = true;
                                            }
                                            else if (data.Joint == JointType.KneeRight)
                                            {
                                                foundlr = true;
                                            }
                                            else if (data.Joint == JointType.KneeLeft)
                                            {
                                                foundll = true;
                                            }
                                            else if (data.Joint == JointType.ElbowRight)
                                            {
                                                foundar = true;
                                            }
                                            else if (data.Joint == JointType.KneeLeft)
                                            {
                                                foundal = true;
                                            }
                                        }
                                    }
                                    if (!foundn)
                                    {
                                        joint_speed_data.Add(new JointSpeed(JointType.Neck,item.Item3));
                                    }
                                    if (!foundar)
                                    {
                                        joint_speed_data.Add(new JointSpeed(JointType.ElbowRight, item.Item3));
                                    }
                                    if (!foundal)
                                    {
                                        joint_speed_data.Add(new JointSpeed(JointType.ElbowLeft, item.Item3));
                                    }
                                    if (!foundlr)
                                    {
                                        joint_speed_data.Add(new JointSpeed(JointType.KneeRight, item.Item3));
                                    }
                                    if (!foundll)
                                    {
                                        joint_speed_data.Add(new JointSpeed(JointType.KneeLeft, item.Item3));
                                    }
                                    int index = properties_list.IndexOf(GetBodyData(GetTrackingIdByBoneLength(item.Item3)));
                                    if (Math.Abs(JointXSpeed(item.Item3,JointType.Neck))>neck_speed_limit || Math.Abs(JointYSpeed(item.Item3, JointType.Neck)) > neck_speed_limit || Math.Abs(JointZSpeed(item.Item3, JointType.Neck)) > neck_speed_limit)
                                    {
                                        properties_list[index] = new BodyProperties(properties_list[index].JointsLocationData, properties_list[index].JointsOrganizationData, properties_list[index].IsTracked, properties_list[index].IsRestricted, properties_list[index].LeanTrackingState, properties_list[index].XLean, properties_list[index].XLean, properties_list[index].RightHandState, properties_list[index].LeftHandState, properties_list[index].RightHandConfidence, properties_list[index].LeftHandConfidence, false, true,true, true,true, GetTrackingIdByBoneLength(item.Item3));
                                    }
                                    if (Math.Abs(JointXSpeed(item.Item3, JointType.ElbowRight)) > hand_speed_limit || Math.Abs(JointYSpeed(item.Item3, JointType.ElbowRight)) > hand_speed_limit || Math.Abs(JointZSpeed(item.Item3, JointType.ElbowRight)) > hand_speed_limit)
                                    {
                                        properties_list[index] = new BodyProperties(properties_list[index].JointsLocationData, properties_list[index].JointsOrganizationData, properties_list[index].IsTracked, properties_list[index].IsRestricted, properties_list[index].LeanTrackingState, properties_list[index].XLean, properties_list[index].XLean, properties_list[index].RightHandState, properties_list[index].LeftHandState, properties_list[index].RightHandConfidence, properties_list[index].LeftHandConfidence, true, true, true, false, true, GetTrackingIdByBoneLength(item.Item3));
                                    }
                                    if (Math.Abs(JointXSpeed(item.Item3, JointType.ElbowLeft)) > hand_speed_limit || Math.Abs(JointYSpeed(item.Item3, JointType.ElbowLeft)) > hand_speed_limit || Math.Abs(JointZSpeed(item.Item3, JointType.ElbowLeft)) > hand_speed_limit)
                                    {
                                        properties_list[index] = new BodyProperties(properties_list[index].JointsLocationData, properties_list[index].JointsOrganizationData, properties_list[index].IsTracked, properties_list[index].IsRestricted, properties_list[index].LeanTrackingState, properties_list[index].XLean, properties_list[index].XLean, properties_list[index].RightHandState, properties_list[index].LeftHandState, properties_list[index].RightHandConfidence, properties_list[index].LeftHandConfidence, true, true, true, true, false, GetTrackingIdByBoneLength(item.Item3));
                                    }
                                    if (Math.Abs(JointXSpeed(item.Item3, JointType.KneeRight)) > leg_speed_limit || Math.Abs(JointYSpeed(item.Item3, JointType.KneeRight)) > leg_speed_limit || Math.Abs(JointZSpeed(item.Item3, JointType.KneeRight)) > leg_speed_limit)
                                    {
                                        properties_list[index] = new BodyProperties(properties_list[index].JointsLocationData, properties_list[index].JointsOrganizationData, properties_list[index].IsTracked, properties_list[index].IsRestricted, properties_list[index].LeanTrackingState, properties_list[index].XLean, properties_list[index].XLean, properties_list[index].RightHandState, properties_list[index].LeftHandState, properties_list[index].RightHandConfidence, properties_list[index].LeftHandConfidence, true, false, true, true, true, GetTrackingIdByBoneLength(item.Item3));
                                    }
                                    if (Math.Abs(JointXSpeed(item.Item3, JointType.KneeLeft)) > leg_speed_limit || Math.Abs(JointYSpeed(item.Item3, JointType.KneeLeft)) > leg_speed_limit || Math.Abs(JointZSpeed(item.Item3, JointType.KneeLeft)) > leg_speed_limit)
                                    {
                                        properties_list[index] = new BodyProperties(properties_list[index].JointsLocationData, properties_list[index].JointsOrganizationData, properties_list[index].IsTracked, properties_list[index].IsRestricted, properties_list[index].LeanTrackingState, properties_list[index].XLean, properties_list[index].XLean, properties_list[index].RightHandState, properties_list[index].LeftHandState, properties_list[index].RightHandConfidence, properties_list[index].LeftHandConfidence, true, true, false, true, true, GetTrackingIdByBoneLength(item.Item3));
                                    }
                                }
                            }
                            if (calc_bone_id)
                            {
                                foreach (var item in properties_list)
                                {
                                    if (item.IsTracked)
                                    {
                                        double bone_lenght = BoneLength(item.JointsLocationData[JointType.SpineBase], item.JointsLocationData[JointType.SpineMid]);
                                        bool found = false;
                                        for (int i = 0; i < bone_id.Count; i++)
                                        {
                                            if (bone_id[i].Item3 == 0)
                                            {
                                                found = true;
                                            }
                                            else
                                            {
                                                if (bone_id[i].Item3 != 0 && bone_id[i].Item3 > bone_lenght - bonelength_telorance && bone_id[i].Item3 < bone_lenght + bonelength_telorance)
                                                {
                                                    bone_id[i] = new Tuple<int, string, double>(item.TrackingId, bone_id[i].Item2, bone_lenght);
                                                    found = true;
                                                    break;
                                                }
                                            }
                                        }
                                        if (!found)
                                        {
                                            bone_id.Add(new Tuple<int, string, double>(item.TrackingId, null, bone_lenght));
                                        }
                                    }
                                }
                            }
                            if (BodiesDataReceivedEvent != null && properties_list != null)
                            {
                                BodiesDataReceivedEvent.Invoke(null);
                            }
                        }
                    }
                }
            }
            public class BodiesDataReceivedEventArgs : EventArgs
            {
                public List<BodyProperties> Properties;
                public BodiesDataReceivedEventArgs(List<BodyProperties> properties)
                {
                    Properties = properties;
                }
            }
            public List<Tuple<int, string, double>> TrackedBoneID()
            {
                List<Tuple<int, string, double>> to_return = new List<Tuple<int, string, double>>();
                foreach (var item in bone_id)
                {
                    if (IsTracked(item.Item1))
                    {
                        to_return.Add(new Tuple<int, string, double>(item.Item1, item.Item2, item.Item3));
                    }
                }
                return to_return;
            }
            public bool IsLegalArmRight(double bone)
            {
                return GetBodyData(GetTrackingIdByBoneLength(bone)).IsLegalArmRight;
            }
            public bool IsLegalArmLeft(double bone)
            {
                return GetBodyData(GetTrackingIdByBoneLength(bone)).IsLegalArmLeft;
            }
            public bool IsLegalHead(double bone)
            {
                return GetBodyData(GetTrackingIdByBoneLength(bone)).IsLegalHead;
            }
            public bool IsLegalLegRight(double bone)
            {
                return GetBodyData(GetTrackingIdByBoneLength(bone)).IsLegalLegRight;
            }
            public bool IsLegalLegLeft(double bone)
            {
                return GetBodyData(GetTrackingIdByBoneLength(bone)).IsLegalLegLeft;
            }
            public bool IsTracked(int tracking_id)
            {
                if (tracking_id == -1)
                {
                    return false;
                }
                return GetBodyData(tracking_id).IsTracked;
            }
            public bool IsRestricted(int tracking_id)
            {
                return GetBodyData(tracking_id).IsRestricted;
            }
            public float XLean(int tracking_id)
            {
                return GetBodyData(tracking_id).XLean;
            }
            public float YLean(int tracking_id)
            {
                return GetBodyData(tracking_id).YLean;
            }
            public float JointXSpeed(double bone, JointType joint)
            {
                foreach (var item in joint_speed_data)
                {
                    if (IsBoneMatch(bone,item.Bone) && joint == item.Joint)
                    {
                        return item.XSpeed();
                    }
                }
                return -1;
            }
            public float JointYSpeed(double bone, JointType joint)
            {
                foreach (var item in joint_speed_data)
                {
                    if (IsBoneMatch(bone, item.Bone) && joint == item.Joint)
                    {
                        return item.YSpeed();
                    }
                }
                return -1;
            }
            public float JointZSpeed(double bone, JointType joint)
            {
                foreach (var item in joint_speed_data)
                {
                    if (IsBoneMatch(bone, item.Bone) && joint == item.Joint)
                    {
                        return item.ZSpeed();
                    }
                }
                return -1;
            }
            public static bool IsBoneMatch(double bone, double source_bone)
            {
                if (bone < 0 || source_bone < 0)
                {
                    return false;
                }
                if (bone > source_bone - bonelength_telorance && bone < source_bone + bonelength_telorance)
                {
                    return true;
                }
                return false;
            }
            public List<JointType> GetAllJointsNames(BodyProperties properties)
            {
                List<JointType> to_return = new List<JointType>();
                foreach (var item in properties.JointsLocationData.Keys)
                {
                    to_return.Add(item);
                }
                return to_return;
            }
            public HandState RightHandState(int tracking_id)
            {
                return GetBodyData(tracking_id).RightHandState;
            }
            public HandState LeftHandState(int tracking_id)
            {
                return GetBodyData(tracking_id).LeftHandState;
            }
            public TrackingState LeanTrackingState(int tracking_id)
            {
                return GetBodyData(tracking_id).LeanTrackingState;
            }
            public TrackingConfidence RightHandConfidence(int tracking_id)
            {
                return GetBodyData(tracking_id).RightHandConfidence;
            }
            public TrackingConfidence LeftHandConfidence(int tracking_id)
            {
                return GetBodyData(tracking_id).LeftHandConfidence;
            }
            public CameraSpacePoint JointPosition(int tracking_id, JointType joint)
            {
                return GetBodyData(tracking_id).JointsLocationData[joint].Position;
            }
            public Vector4 JointOrganization(int tracking_id, JointType joint)
            {
                return GetBodyData(tracking_id).JointsOrganizationData[joint].Orientation;
            }
            public TrackingState JointTrackingState(int tracking_id, JointType joint)
            {
                return GetBodyData(tracking_id).JointsLocationData[joint].TrackingState;
            }
            public double BoneLength(JointType joint1, JointType joint2, int tracking_id)
            {
                Joint j1 = new Joint(), j2 = new Joint(); bool found = false;
                foreach (var item in properties_list)
                {
                    if (item.TrackingId == tracking_id)
                    {
                        j1 = item.JointsLocationData[joint1];
                        j2 = item.JointsLocationData[joint2];
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    return Math.Sqrt(Math.Pow(j1.Position.X - j2.Position.X, 2) + Math.Pow(j1.Position.Y - j2.Position.Y, 2) + Math.Pow(j1.Position.Z - j2.Position.Z, 2));
                }
                throw new RobonikException("Tracking id isn`t valid.");
            }
            double BoneLength(Joint joint1, Joint joint2)
            {
                return Math.Sqrt(Math.Pow(joint1.Position.X - joint2.Position.X, 2) + Math.Pow(joint1.Position.Y - joint2.Position.Y, 2) + Math.Pow(joint1.Position.Z - joint2.Position.Z, 2));
            }
            public BodyProperties GetBodyData(int trackingid)
            {
                foreach (var item in properties_list)
                {
                    if (item.TrackingId == trackingid)
                    {
                        return item;
                    }
                }
                return new BodyProperties(null, null, false, false, TrackingState.NotTracked, 0, 0, HandState.NotTracked, HandState.NotTracked, TrackingConfidence.High, TrackingConfidence.High,false,false,false,false,false, -1);
            }
            public int GetTrackingIdByBoneLength(double bone_lenght)
            {
                foreach (var item in bone_id)
                {
                    if (item.Item3 > bone_lenght - bonelength_telorance && item.Item3 < bone_lenght + bonelength_telorance)
                    {
                        return item.Item1;
                    }
                }
                return -1;
            }
            public int GetTrackingIdByName(string name)
            {
                foreach (var item in bone_id)
                {
                    if (item.Item2 == name)
                    {
                        return item.Item1;
                    }
                }
                return -1;
            }
            public double GetBoneLengthByName(string name)
            {
                foreach (var item in bone_id)
                {
                    if (item.Item2 == name)
                    {
                        return item.Item3;
                    }
                }
                throw new RobonikException("Bone lenght for this name is not recognized.");
            }
            public double GetBoneLengthByTrackingId(int tracking_id)
            {
                foreach (var item in bone_id)
                {
                    if (tracking_id == item.Item1)
                    {
                        return item.Item3;
                    }
                }
                throw new RobonikException("Bone lenght for this name is not recognized.");
            }
            public bool BoneIDRecored
            {
                get
                {
                    return calc_bone_id;
                }
                set
                {
                    calc_bone_id = value;
                }
            }
            public bool DataStoration
            {
                get
                {
                    return datastoration;
                }
                set
                {
                    datastoration = value;
                }
            }
            public int BodyCount
            {
                get
                {
                    return body_count;
                }

                private set
                {
                    body_count = value;
                }
            }
            public bool AccurateHand
            {
                get
                {
                    return accurate_hand;
                }
                set
                {
                    accurate_hand = value;
                }
            }
            public bool DropIllegalData
            {
                get
                {
                    return illegaldrop;
                }
                set
                {
                    illegaldrop = value;
                }
            }
            public List<Tuple<int, string, double>> BoneID
            {
                get
                {
                    return bone_id;
                }
            }
            public bool Pause
            {
                get
                {
                    if (frame_reader != null)
                    {
                        return frame_reader.IsPaused;
                    }
                    return false;
                }
                set
                {
                    if (frame_reader != null)
                    {
                        frame_reader.IsPaused = value;
                    }
                }
            }
        }
        //To Test (Speach Recognition Incomplete)
        public class Audio : KinectIntraction
        {
            AudioBeamFrameReader frame_reader = null;List<VoiceData> voice_data;bool calc_voice = false;
            public event RobonicEventHandler VoiceReceivedEvent;
            //bool1: be opened, is opened
            List<Tuple<double,bool,bool>> voice_recognition_list;
            public struct VoiceData
            {
                public double BeamAngle;public float Confidence;public double Bone;
                public VoiceData(double angle, float confidence, double bone)
                {
                    BeamAngle = angle;
                    Confidence = confidence;
                    Bone = bone;
                }
            }
            public Audio()
            {
                if (Sensor.IsAvailable)
                {
                    voice_data = new List<VoiceData>();
                    voice_recognition_list = new List<Tuple<double, bool,bool>>();
                    frame_reader = Sensor.AudioSource.OpenReader();
                    frame_reader.FrameArrived += Frame_reader_FrameArrived;
                }
                else
                {
                    new RobonikException("Sensor is not available.");
                }
            }
            private void Frame_reader_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
            {
                if (calc_voice || VoiceReceivedEvent != null)
                {
                    using (AudioBeamFrameList framelist = e.FrameReference.AcquireBeamFrames())
                    {
                        if (framelist != null)
                        {
                            foreach (var frame in framelist)
                            {
                                IReadOnlyList<AudioBeamSubFrame> subframelist = frame.SubFrames;
                                foreach (var subframe in subframelist)
                                {
                                    IReadOnlyList<AudioBodyCorrelation> bodydata = subframe.AudioBodyCorrelations;
                                    foreach (var data in bodydata)
                                    {
                                        //foreach (var item in voice_recognition_list)
                                        //{
                                        //    if ( && BodyData.IsBoneMatch(item.Item1, body_data.GetBoneLengthByTrackingId((int)data.BodyTrackingId)))
                                        //    {
                                        //        Stream stream = frame.AudioBeam.OpenInputStream();
                                        //        KinectAudioStream converted_stream = new KinectAudioStream(stream);
                                        //    }
                                        //}
                                        int length = voice_data.Count;
                                        bool found = false;
                                        for (int i = 0; i < length; i++)
                                        {
                                            if (BodyData.IsBoneMatch(body_data.GetBoneLengthByTrackingId((int)data.BodyTrackingId), voice_data[i].Bone))
                                            {
                                                if (subframe.BeamAngleConfidence >= voice_data[i].Confidence)
                                                {
                                                    voice_data[i] = new VoiceData(subframe.BeamAngle, subframe.BeamAngleConfidence, body_data.GetBoneLengthByTrackingId((int)data.BodyTrackingId));
                                                    found = true;
                                                    break;
                                                }
                                            }
                                        }
                                        if (!found)
                                        {
                                            voice_data.Add(new VoiceData(subframe.BeamAngle, subframe.BeamAngleConfidence, body_data.GetBoneLengthByTrackingId((int)data.BodyTrackingId)));
                                        }
                                    }
                                }
                            }
                            if (VoiceReceivedEvent != null)
                            {
                                VoiceReceivedEvent.Invoke(null);
                            }
                        }
                    }
                }
            }
            public List<VoiceData> VoiceDataList
            {
                get
                {
                    return voice_data;
                }
            }
            public float GetAngleConfidence(double bone)
            {
                foreach (var item in voice_data)
                {
                    if (BodyData.IsBoneMatch(item.Bone,bone))
                    {
                        return item.Confidence;
                    }
                }
                return -1;
            }
            public double GetAngle(double bone)
            {
                foreach (var item in voice_data)
                {
                    if (BodyData.IsBoneMatch(item.Bone, bone))
                    {
                        return item.BeamAngle;
                    }
                }
                return -1;
            }
            BodyData body_data
            {
                get
                {
                    return GetBodyData.Get();
                }
            }
            public bool DataStoration
            {
                get
                {
                    return calc_voice;
                }
                set
                {
                    calc_voice = value;
                }
            }
            public void AddToSpeechRecognitionList(double bone)
            {
                foreach (var item in voice_recognition_list)
                {
                    if (BodyData.IsBoneMatch(bone,item.Item1))
                    {
                        return;
                    }
                }
                voice_recognition_list.Add(new Tuple<double, bool, bool>(bone,true,false));
            }
            public void RemoveFromSpeechRecognitionList(double bone)
            {
                int length = voice_recognition_list.Count;
                for (int i = 0; i < length; i++)
                {
                    if (voice_recognition_list[i].Item3 && BodyData.IsBoneMatch(bone, voice_recognition_list[i].Item1))
                    {
                        voice_recognition_list[i] = new Tuple<double, bool, bool>(voice_recognition_list[i].Item1, false,true);
                    }
                }
            }
            public bool Pause
            {
                get
                {
                    if (frame_reader != null)
                    {
                        return frame_reader.IsPaused;
                    }
                    return false;
                }
                set
                {
                    if (frame_reader != null)
                    {
                        frame_reader.IsPaused = value;
                    }
                }
            }
        }
        internal class KinectAudioStream : Stream
        {
            private Stream kinect32BitStream;
            public KinectAudioStream(Stream input)
            {
                this.kinect32BitStream = input;
            }
            public bool SpeechActive { get; set; }
            public override bool CanRead
            {
                get { return true; }
            }
            public override bool CanWrite
            {
                get { return false; }
            }
            public override bool CanSeek
            {
                get { return false; }
            }
            public override long Position
            {
                get { return 0; }
                set { throw new NotImplementedException(); }
            }
            public override long Length
            {
                get { throw new NotImplementedException(); }
            }
            public override void Flush()
            {
                throw new NotImplementedException();
            }
            public override long Seek(long offset, SeekOrigin origin)
            {
                return 0;
            }
            public override void SetLength(long value)
            {
                throw new NotImplementedException();
            }
            public override void Write(byte[] buffer, int offset, int count)
            {
                throw new NotImplementedException();
            }
            public override int Read(byte[] buffer, int offset, int count)
            {
                const int SampleSizeRatio = sizeof(float) / sizeof(short); // = 2. 
                const int SleepDuration = 50;
                int readcount = count * SampleSizeRatio;
                byte[] kinectBuffer = new byte[readcount];
                int bytesremaining = readcount;
                while (bytesremaining > 0)
                {
                    if (!this.SpeechActive)
                    {
                        return 0;
                    }
                    int result = this.kinect32BitStream.Read(kinectBuffer, readcount - bytesremaining, bytesremaining);
                    bytesremaining -= result;
                    if (bytesremaining > 0)
                    {
                        System.Threading.Thread.Sleep(SleepDuration);
                    }
                }
                for (int i = 0; i < count / sizeof(short); i++)
                {
                    float sample = BitConverter.ToSingle(kinectBuffer, i * sizeof(float));
                    if (sample > 1.0f)
                    {
                        sample = 1.0f;
                    }
                    else if (sample < -1.0f)
                    {
                        sample = -1.0f;
                    }
                    short convertedSample = Convert.ToInt16(sample * short.MaxValue);
                    byte[] local = BitConverter.GetBytes(convertedSample);
                    Buffer.BlockCopy(local, 0, buffer, offset + (i * sizeof(short)), sizeof(short));
                }
                return count;
            }
        }
        //Completed
        public static class GetBodyData
        {
            static BodyData data = null;
            public static BodyData Get()
            {
                if (data == null)
                {
                    data = new BodyData();
                }
                return data;
            }
        }
        //To Test
        public class Speaker
        {
            SpeechSynthesizer reader; bool is_speaking = false; List<SpeakingData> list;
            public event RobonicEventHandler SpeackingCompleted;int queue = 0;
            struct SpeakingData
            {
                public string Text; public Prompt Prompt;public bool Paused;
                public SpeakingData(string text, Prompt prompt)
                {
                    Text = text;
                    Prompt = prompt;
                    Paused = false;
                }
            }
            public class SpeackingCompletedEventArgs : EventArgs
            {
                string Text;bool Canceled;
                public SpeackingCompletedEventArgs(string text, bool canceled)
                {
                    Text = text;
                    Canceled = canceled;
                }
            }
            public Speaker()
            {
                list = new List<SpeakingData>();
                reader = new SpeechSynthesizer();
                IReadOnlyCollection<InstalledVoice> installed = reader.GetInstalledVoices();
                if (installed != null && installed.Count != 0)
                {
                    foreach (var item in installed)
                    {
                        if (item.Enabled && item.VoiceInfo.Gender == VoiceGender.Male)
                        {
                            reader.SelectVoice(item.VoiceInfo.Name);
                            break;
                        }
                    }
                    reader.SpeakCompleted += Reader_SpeakCompleted;
                }
                else
                {
                    throw new RobonikException("No voice is installed.");
                }
            }
            public void Speak(string text)
            {
                list.Add(new SpeakingData(text, new Prompt(text)));
                reader.SpeakAsync(list[list.Count - 1].Prompt);
                is_speaking = true;
                queue++;
            }
            public void CancelSpeak(string text)
            {
                if (is_speaking)
                {
                    foreach (var item in list)
                    {
                        if (item.Text == text)
                        {
                            reader.SpeakAsyncCancel(item.Prompt);
                        }
                    }
                }
            }
            public void MakeSilent()
            {
                reader.SpeakAsyncCancelAll();
            }
            private void Reader_SpeakCompleted(object sender, SpeakCompletedEventArgs e)
            {
                is_speaking = false;
                queue--;
                if (SpeackingCompleted != null)
                {
                    SpeackingCompleted.Invoke(new SpeackingCompletedEventArgs(list[0].Text,e.Cancelled));
                }
                list.Remove(list[0]);
                if (list.Count != 0)
                {
                    Speak(list[0].Text);
                }
            }
            public void Pause()
            {
                reader.Pause();
            }
            public void Resume()
            {
                reader.Resume();
            }
            public bool IsSpeaking
            {
                get
                {
                    return is_speaking;
                }
            }
            public int Queue
            {
                get
                {
                    return queue;
                }
            }
            public int Volume
            {
                get
                {
                    return reader.Volume;
                }
                set
                {
                    reader.Volume = value;
                }
            }
        }
        //not completed
        public class Face : KinectIntraction
        {
            FaceFrameReader frame_reader;FaceFrameSource frame_source;public event RobonicEventHandler FaceDisappearedEvent;double bone;
            public class FaceDisappearedEventArgs : EventArgs
            {
                double Bone;
                public FaceDisappearedEventArgs(double bone)
                {
                    Bone = bone;
                }
            }
            public Face(double user_bone)
            {
                bone = user_bone;
                bool invoke = false;
                if (Sensor.IsAvailable)
                {
                    ulong tracking_id = 0;
                    try
                    {
                        tracking_id = (ulong)body_data.GetTrackingIdByBoneLength(bone);
                    }
                    catch (Exception)
                    {
                        invoke = true;
                    }
                    frame_source = new FaceFrameSource(Sensor, tracking_id,
                    FaceFrameFeatures.FaceEngagement |
                    FaceFrameFeatures.Glasses |
                    FaceFrameFeatures.Happy |
                    FaceFrameFeatures.LeftEyeClosed |
                    FaceFrameFeatures.LookingAway |
                    FaceFrameFeatures.MouthMoved |
                    FaceFrameFeatures.MouthOpen |
                    FaceFrameFeatures.RightEyeClosed |
                    FaceFrameFeatures.RotationOrientation);
                    frame_reader = frame_source.OpenReader();
                    frame_source.TrackingIdLost += Frame_source_TrackingIdLost;
                    frame_reader.FrameArrived += Frame_reader_FrameArrived;
                    if (!frame_source.IsTrackingIdValid)
                    {
                        invoke = true;
                    }
                    if (invoke)
                    {
                        if (FaceDisappearedEvent != null)
                        {
                            FaceDisappearedEvent.Invoke(new FaceDisappearedEventArgs(bone));
                        }
                    }
                }
                else
                {
                    if (FaceDisappearedEvent!=null)
                    {
                        FaceDisappearedEvent.Invoke(new FaceDisappearedEventArgs(bone));
                    }
                }
            }
            void objects_disposer()
            {
                if (frame_reader != null)
                {
                    //if (frame_reader.FrameArrived != null)
                    //{

                    //}
                }
            }

            private void Frame_source_TrackingIdLost(object sender, Microsoft.Kinect.Face.TrackingIdLostEventArgs e)
            {
                if (body_data.GetTrackingIdByBoneLength(bone) == -1)
                {

                }
            }

            private void Frame_reader_FrameArrived(object sender, FaceFrameArrivedEventArgs e)
            {
                throw new NotImplementedException();
            }

            BodyData body_data
            {
                get
                {
                    return GetBodyData.Get();
                }
            }
            public void Cancel()
            {
                if (frame_reader != null)
                {
                    
                }
            }
            public bool Pause
            {
                get
                {
                    if (frame_reader != null)
                    {
                        return frame_reader.IsPaused;
                    }
                    return false;
                }
                set
                {
                    if (frame_reader != null)
                    {
                        frame_reader.IsPaused = value;
                    }
                }
            }
        }
    }
}
