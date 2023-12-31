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

public class ROSSingleTransformPublisher: MonoBehaviour
{
    const string k_TfTopic = "/tf";
    
    [SerializeField]
    double m_PublishRateHz = 20f;
    [SerializeField]
    /* List<string> m_GlobalFrameIds = new List<string> { "map", "odom" }; */
    public string origFrame;
    public string secondObjectFrame;
    public GameObject secondObject;
    
    double m_LastPublishTimeSeconds;

    ROSConnection m_ROS;

    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    private GameObject follower;

    // Start is called before the first frame update
    void Start()
    {

        m_ROS = ROSConnection.GetOrCreateInstance();
        m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
        follower = transform.GetChild(0).gameObject;
    }


    void PublishMessage()
    {
        var tfMessageList = new List<TransformStampedMsg>();

        /* if (m_GlobalFrameIds.Count > 0) */
        /* { */
        /*     var tfRootToGlobal = new TransformStampedMsg( */
        /*         new HeaderMsg(new TimeStamp(Clock.time), m_GlobalFrameIds.Last()), */
        /*         m_TransformRoot.name, */
        /*         m_TransformRoot.Transform.To<FLU>()); */
        /*     tfMessageList.Add(tfRootToGlobal); */
        /* } */
        /* else */
        /* { */
        /*     Debug.LogWarning($"No {m_GlobalFrameIds} specified, transform tree will be entirely local coordinates."); */
        /* } */
        
        /* // In case there are multiple "global" transforms that are effectively the same coordinate frame, */ 
        /* // treat this as an ordered list, first entry is the "true" global */
        /* for (var i = 1; i < m_GlobalFrameIds.Count; ++i) */
        /* { */
        /*     var tfGlobalToGlobal = new TransformStampedMsg( */
        /*         new HeaderMsg(new TimeStamp(Clock.time), m_GlobalFrameIds[i - 1]), */
        /*         m_GlobalFrameIds[i], */
        /*         // Initializes to identity transform */
        /*         new TransformMsg()); */
        /*     tfMessageList.Add(tfGlobalToGlobal); */
        /* } */

        /* PopulateTFList(tfMessageList, m_TransformRoot); */

        /* var tfGlobalToGlobal = new TransformStampedMsg( */
        /*     new HeaderMsg(new TimeStamp(Clock.time), m_GlobalFrameIds[i - 1]), */
        /*     m_GlobalFrameIds[i], */
        /*     // Initializes to identity transform */
        /*     new TransformMsg()); */

        var header = new HeaderMsg();
        float time = (float)Clock.time;

        uint sec = (uint)Math.Truncate(time);
        uint nanosec = (uint)( (time - sec)*1e+9 );
        header.stamp.sec = sec;
        header.stamp.nanosec = nanosec;
        header.frame_id = origFrame;

        var transformmsg = new TransformStampedMsg(header, 
            secondObjectFrame, 
            follower.transform.ToROSTransform());

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
            follower.transform.position = secondObject.transform.position;
            follower.transform.rotation = secondObject.transform.rotation;
            PublishMessage();
        }

    }
}
