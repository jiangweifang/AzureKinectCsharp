// See https://aka.ms/new-console-template for more information
using Microsoft.Azure.Kinect.Sensor;
using Microsoft.Azure.Kinect.BodyTracking;
using System.Numerics;
internal class Program
{
    static void Main(string[] args)
    {
        Console.WriteLine("Hello, World!");
        var k4a = Device.Open();
        try
        {
            var config = new DeviceConfiguration();
            config.ColorResolution = ColorResolution.Off;
            config.DepthMode = DepthMode.NFOV_Unbinned;
            Console.WriteLine(k4a.SerialNum);
            k4a.StartCameras(config);//开始
            var cal = k4a.GetCalibration(DepthMode.NFOV_Unbinned, ColorResolution.Off);
            //var imgCap = k4a.GetCapture();
            var tracker = Tracker.Create(cal, TrackerConfiguration.Default);
            Console.WriteLine(tracker);
            int frame_count = 0;
            do
            {
                frame_count++;
                var get_capture_result = k4a.GetCapture();
                tracker.EnqueueCapture(get_capture_result);
                var pop_frame_result = tracker.PopResult();
                Console.WriteLine("有{0}个人在这里",pop_frame_result.NumberOfBodies);
                if (pop_frame_result.NumberOfBodies >= 1)
                {
                    //body 0 代表距离前方最近的人
                    var body = pop_frame_result.GetBody(0);
                    //右肩膀在三维向量中的点
                    var shoulderRight = body.Skeleton.GetJoint(JointId.ShoulderRight);
                    //紧挨着右肩膀坐标的关节是右肘部  在三维向量中的点
                    var elbowRight = body.Skeleton.GetJoint(JointId.ElbowRight);
                    //右手腕 在三维向量中的点
                    var wristRight = body.Skeleton.GetJoint(JointId.WristRight);

                    Console.WriteLine("右肘关节与大臂之间的角度是{0}°", Angle(shoulderRight.Position, elbowRight.Position, wristRight.Position));

                }
                Thread.Sleep(100);
            } while (frame_count < 1000);
            k4a.StopCameras();
            k4a.Dispose();
        }
        catch (Exception ex)
        {
            Console.WriteLine(ex.Message);
            if (k4a != null)
                k4a.Dispose();
        }
        finally
        {
            if (k4a != null)
                k4a.Dispose();
        }
    }
    private static float Angle(Vector3 start , Vector3 pass ,Vector3 end)
    {
        var line1 = Vector3.Distance(start, pass);
        var line2 = Vector3.Distance(pass, end);
        var refLine = Vector3.Distance(end, start);
        return Triangle(line1 , line2 , refLine);
    }

    private static float Triangle(float line1, float line2, float refLine)
    {
        var reg = MathF.Acos((line1 * line1 + line2 * line2 - refLine * refLine) / (2 * line1 * line2));
        return reg * (180 / MathF.PI);
    }
}

