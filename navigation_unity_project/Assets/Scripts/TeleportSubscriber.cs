using System;
using System.Collections.Generic;
/* using System.Linq; */
/* using RosMessageTypes.Geometry; */
/* using RosMessageTypes.Std; */
/* using RosMessageTypes.Tf2; */
/* using Unity.Robotics.Core; */
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

using PoseMsg = RosMessageTypes.Geometry.PoseMsg;

public class TeleportSubscriber : MonoBehaviour
{
    public String topic_name = "/robot1/chassis_link/teleport";
    // Start is called before the first frame update
    void Start()
    {
        /* m_ROS = ROSConnection.GetOrCreateInstance(); */
        ROSConnection.GetOrCreateInstance().Unsubscribe(topic_name);
        ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>(topic_name, Teleport);
    }
    /* void OnDestroy(){ */
    /*     ROSConnection.GetOrCreateInstance().Unsubscribe(topic_name); */
    /* } */

    // Update is called once per frame
    void Update()
    {
        
    }

    void Teleport(PoseMsg msg){
      Vector3 pos;
      pos.x = (float)msg.position.x;
      pos.y = (float)msg.position.y;
      pos.z = (float)msg.position.z;
      pos = FLU.ConvertToRUF(pos);
      Quaternion q;
      q.x = (float)msg.orientation.x;
      q.y = (float)msg.orientation.y;
      q.z = (float)msg.orientation.z;
      q.w = (float)msg.orientation.w;
      q = FLU.ConvertToRUF(q);

      if(GetComponent<ArticulationBody>()){
        GetComponent<ArticulationBody>().TeleportRoot(pos, q);
      }
      else{
        transform.position = pos;
        transform.rotation = q;
      }
    }

}
