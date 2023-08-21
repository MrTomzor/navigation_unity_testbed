using System.Collections;
using System.Collections.Generic;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
/* using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg; */
using TwistStampedMsg = RosMessageTypes.Geometry.TwistStampedMsg;

public class VelocityCommandRosInterface : MonoBehaviour
{
  public string ref_topic_name;
  /* public string encoder_topic_name; */
  public float motor_force = 10;
  public float motor_torque = 10;
  public float ground_raycast_len = .75f;

  public Vector3 ref_linear;
  public Vector3 ref_rotation;

  public bool enableGameControl = false;
  public float mouseSensitivity = 0.8f;
  public float linearGameControlSensitivity = 10;

  public bool velocityControl = true; //alternative is forcecontrol

  void Start()
  {
    ROSConnection.GetOrCreateInstance().Subscribe<TwistStampedMsg >(ref_topic_name, SetReferenceVelocity);
  }

  // Update is called once per frame
  void Update()
  {
    if(enableGameControl){
      Screen.lockCursor = true;
      ref_linear = new Vector3(Input.GetAxis("Horizontal"), (Input.GetKey(KeyCode.Space) ? 1 : 0 ) - (Input.GetKey("c") ? 1 : 0), Input.GetAxis("Vertical"));
      ref_rotation = new Vector3(-Input.GetAxis("Mouse Y"),Input.GetAxis("Mouse X"),(Input.GetKey("q") ? 1 : 0 ) - (Input.GetKey("e") ? 1 : 0) );
      ref_linear *= linearGameControlSensitivity;
      if(Input.GetKey(KeyCode.LeftShift)){
        ref_linear *= 2;
      }
      ref_rotation *= mouseSensitivity;
    }
    else{
      Screen.lockCursor = false;
    }

    var rb = GetComponent<Rigidbody>();
    if(velocityControl){
      /* VEL CONTROL */

      //Vector3 current_velocity = rb.velocity;
      Vector3 current_velocity  = transform.InverseTransformDirection(rb.velocity);
      float current_angular_velocity = rb.angularVelocity.y;

      /* float angvel_error = ref_rotation.z - current_angular_velocity; */
      Vector3 angvel_error = ref_rotation - transform.InverseTransformDirection(rb.angularVelocity);
      /* Vector3 angvel_error = ref_rotation - rb.angularVelocity; */

      Vector3 linear_error = ref_linear - current_velocity;

      //rb.AddTorque(transform.up * (float)(msg.twist.angular.z * motor_torque));
      /* float torque = (float)(angvel_error * motor_torque); */
      /* float torque = (float)(angvel_error.magnitude * motor_torque); */
      /* rb.AddTorque(transform.up * torque); */

      rb.AddTorque(transform.TransformDirection(angvel_error * motor_torque));
      rb.AddForce(transform.TransformDirection(linear_error * motor_force));
    }
    else{
      /* FORCE CONTROL */
      rb.AddTorque(transform.TransformDirection(ref_rotation * motor_torque));
      rb.AddForce(transform.TransformDirection(ref_linear * motor_force));
    }
  }

  void SetReferenceVelocity(TwistStampedMsg  msg)
  {
    ref_linear.x = -(float)msg.twist.linear.y;
    ref_linear.y = (float)msg.twist.linear.z;
    ref_linear.z = (float)msg.twist.linear.x;

    ref_rotation.x = (float)msg.twist.angular.x;
    ref_rotation.y = (float)msg.twist.angular.y;
    ref_rotation.z = (float)msg.twist.angular.z;
  }

  public void ToggleManualControl(bool b){
    enableGameControl  = b;
  }
}
