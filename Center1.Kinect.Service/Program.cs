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
                    //右肩膀
                    var shoulderRight_joint = body.Skeleton.GetJoint(JointId.ShoulderRight);
                    //右肩膀在二维向量中的点
                    var shoulderRight_dot = shoulderRight_joint.Position;
                    //紧挨着右肩膀坐标的关节是右肘部
                    var elbowRight_joint = body.Skeleton.GetJoint(JointId.ElbowRight);
                    //右肘部在二维向量中的点
                    var elbowRight_dot = elbowRight_joint.Position;
                    //右手腕
                    var wristRight_joint = body.Skeleton.GetJoint(JointId.WristRight);
                    //右手腕在二维向量中的点
                    var wristRight_dot = wristRight_joint.Position;
                    var handRight_joint = body.Skeleton.GetJoint(JointId.HandRight);
                    var spineChest_joint = body.Skeleton.GetJoint(JointId.SpineChest);
                    //计算平面数点距
                    //右臂
                    var armRight = Vector3.Distance(shoulderRight_dot, elbowRight_dot);
                    //右前臂
                    var forearmRight = Vector3.Distance(elbowRight_dot, wristRight_dot);
                    //右侧连线(只有连线才能是三角形)
                    var lineRight = Vector3.Distance(wristRight_dot, shoulderRight_dot);
                    var reg = MathF.Acos((armRight * armRight + forearmRight * forearmRight - lineRight * lineRight) / (2 * armRight * forearmRight));
                    //将返回值乘以 180/ MathF.PI 从弧度转换为度。
                    //Multiply the return value by 180/MathF.PI to convert from radians to degrees.
                    Console.WriteLine("右手的位置:X:{0} Y:{1} Z:{2}", handRight_joint.Position.X, handRight_joint.Position.Y, handRight_joint.Position.Z);
                    Console.WriteLine("脊柱胸部的位置:X:{0} Y:{1} Z:{2}", spineChest_joint.Position.X, spineChest_joint.Position.Y, spineChest_joint.Position.Z);

                    Console.WriteLine("右肘关节与大臂之间的角度是{0}°", reg * (180/MathF.PI));
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
}

