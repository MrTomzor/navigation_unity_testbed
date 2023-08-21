using System.Collections;
using System.Collections.Generic;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
/* using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg; */
using Float32 = RosMessageTypes.Std.Float32Msg;

public class RosWheelInterface : MonoBehaviour
{
    // Start is called before the first frame update
    public string wheel_ref_topic_name;
    public string wheel_encoder_topic_name;
    public float motor_force = 1000;
    /* public GameObject wheel_gameobject; */
    
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Float32>(wheel_ref_topic_name, SetReferenceVelocity);
    }

/*     // Update is called once per frame */
/*     void Update() */
/*     { */
        
/*     } */

    void SetReferenceVelocity(Float32 vel)
    {
      // vel is in degrees/s
      var hinge = GetComponent<HingeJoint>();

      // Make the hinge motor rotate with vel degrees per second and a strong force.
      var motor = hinge.motor;
      motor.force = motor_force;
      motor.targetVelocity = vel.data;
      motor.freeSpin = false;
      hinge.motor = motor;
      hinge.useMotor = true;
      /* cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a); */
    }
}
