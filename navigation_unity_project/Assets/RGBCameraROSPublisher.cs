using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

[RequireComponent(typeof(Camera))]
[RequireComponent(typeof(URP.Sensor.RGBCamera))]
public class RGBCameraROSPublisher : MonoBehaviour
{

  [SerializeField] public string _topicName = "image";
  [SerializeField] private string _infoTopicName;
  [SerializeField] public string _frameId   = "camera";
    
  private float _timeElapsed = 0f;
  private float _timeStamp   = 0f;

  private ROSConnection _ros;
  private CompressedImageMsg _message;    

  private URP.Sensor.RGBCamera _camera;
  public Camera unity_camera;

  public float last_render_time;
    
  void Start()
  {
    // Get camera core component
    this._camera = GetComponent<URP.Sensor.RGBCamera>();
    this._camera.Init();
    this.unity_camera = GetComponent<Camera>();

    // setup ROS
    this._ros = ROSConnection.GetOrCreateInstance();
    this._infoTopicName = this._topicName + "/camera_info";
    this._topicName += "/image/compressed";
    this._ros.RegisterPublisher<CompressedImageMsg>(this._topicName);
    this._ros.RegisterPublisher<CameraInfoMsg>(this._infoTopicName);

    // setup ROS Message
    this._message = new CompressedImageMsg();
    this._message.header.frame_id = this._frameId;
    this._message.format = "jpeg";

    last_render_time = Time.time;
    /* Time.captureFramerate = (int)this._camera._scanRate; */
    Time.captureFramerate = 100;
  }

    void FixedUpdate()
    {
        /* this._timeElapsed += Time.deltaTime; */
        float timenow = Time.time;

        if(timenow > last_render_time + (1f/this._camera.scanRate))
        /* if(this._timeElapsed > (1f/this._camera.scanRate)) */
        {
          last_render_time = timenow;
            /* this._timeElapsed -= (1f/this._camera.scanRate); */
            /* this._timeStamp = Time.time; */
            this._timeStamp = timenow;

            this._camera.ForceUpdateImage(unity_camera);

            // Update ROS Message
            uint sec = (uint)Math.Truncate(this._timeStamp);
            uint nanosec = (uint)( (this._timeStamp - sec)*1e+9 );
            this._message.header.stamp.sec = sec;
            this._message.header.stamp.nanosec = nanosec;
            this._message.data = this._camera.data;
            this._ros.Publish(this._topicName, this._message);

            // Update time
            /* this._timeElapsed = 0; */
            /* this._timeStamp = Time.time; */

            // Send camera info
            CameraInfoMsg info_msg = Unity.Robotics.ROSTCPConnector.MessageGeneration.CameraInfoGenerator.ConstructCameraInfoMessage(this.unity_camera, this._message.header);
            this._ros.Publish(this._infoTopicName, info_msg);
        }
    }
}
