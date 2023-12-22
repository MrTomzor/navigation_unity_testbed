using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

using PoseMsg = RosMessageTypes.Geometry.PoseMsg;

public class ROSGlobalPosePublisher: MonoBehaviour
{
    const string k_TfTopic = "/tf";
    
    [SerializeField]
    double m_PublishRateHz = 20f;
    [SerializeField]
    /* List<string> m_GlobalFrameIds = new List<string> { "map", "odom" }; */
    public string origFrame = "world_origin";
    public string targetFrame;
    
    double m_LastPublishTimeSeconds;

    ROSConnection m_ROS;

    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    // Start is called before the first frame update
    void Start()
    {

        m_ROS = ROSConnection.GetOrCreateInstance();
        /* m_ROS.Subscribe<PoseMsg>(targetFrame + "/teleport", Teleport); */
        m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
    }

    void PublishMessage()
    {
        var tfMessageList = new List<TransformStampedMsg>();

        var header = new HeaderMsg();
        float time = (float)Clock.time;

        uint sec = (uint)Math.Truncate(time);
        uint nanosec = (uint)( (time - sec)*1e+9 );
        header.stamp.sec = sec;
        header.stamp.nanosec = nanosec;
        header.frame_id = origFrame;

        var transformmsg = new TransformStampedMsg(header, 
            targetFrame, 
            transform.ToROSTransformGlobal());

        tfMessageList.Add(transformmsg);

        /* tfMessageList.Add(tfGlobalToGlobal); */

        var tfMessage = new TFMessageMsg(tfMessageList.ToArray());
        m_ROS.Publish(k_TfTopic, tfMessage);
        m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
    }

    void Update()
    {
        if (ShouldPublishMessage)
        {
            PublishMessage();
        }

    }
}
