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
  public float linear_control_p = 10;
  public float linear_control_i = 1;
  public float angular_control_p = 10;
  public float angular_control_i = 1;

  public float ground_raycast_len = .75f;

  public Vector3 ref_linear;
  public Vector3 ref_rotation;

  public bool enableGameControl = false;
  public float mouseSensitivity = 0.8f;
  public float linearGameControlSensitivity = 10;

  public bool velocityControl = true; //alternative is forcecontrol
  public bool carMode = false;

  public Vector3 angvel_error_integral;
  public Vector3 linear_error_integral;
  public float integration_cutoff = 3;

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

    Rigidbody rb = GetComponent<Rigidbody>();
    ArticulationBody ab = GetComponent<ArticulationBody>();

    if(velocityControl){
      /* VEL CONTROL */

      Vector3 current_velocity;
      Vector3 current_angular_velocity;

      if(rb){
        current_velocity  = transform.InverseTransformDirection(rb.velocity);
        current_angular_velocity = transform.InverseTransformDirection(rb.angularVelocity);
      }
      else{
        current_velocity  = transform.InverseTransformDirection(ab.velocity);
        current_angular_velocity = transform.InverseTransformDirection(ab.angularVelocity);
      }
      
      Vector3 angvel_error = ref_rotation - current_angular_velocity;
      Vector3 linear_error = ref_linear - current_velocity;
      if(carMode){
        angvel_error.x = 0;
        angvel_error.z = 0;
        linear_error.y = 0;
        linear_error.x = 0;
      }

      angvel_error_integral += angvel_error;
      linear_error_integral += linear_error;
      if(linear_error.magnitude > integration_cutoff){
        // TODO - do it per element
        linear_error_integral = linear_error_integral * (integration_cutoff / linear_error.magnitude);
      }
      if(angvel_error_integral.magnitude > integration_cutoff){
        // TODO - do it per element
        angvel_error_integral = angvel_error_integral * (integration_cutoff / angvel_error_integral.magnitude);
      }

      if(rb){
        rb.AddTorque(transform.TransformDirection(angvel_error * angular_control_p + angvel_error_integral * angular_control_i));
        rb.AddForce(transform.TransformDirection(linear_error * linear_control_p + linear_error_integral * linear_control_i));
      }
      else{
        ab.AddTorque(transform.TransformDirection(angvel_error * angular_control_p + angvel_error_integral * angular_control_i));
        ab.AddForce(transform.TransformDirection(linear_error * linear_control_p + linear_error_integral * linear_control_i));
      }
      Debug.Log("CURRENT VELOCITY AND ANGVELOCITY:");
      Debug.Log(current_velocity);
      Debug.Log(current_angular_velocity);
    }
    else{
      /* FORCE CONTROL */
      rb.AddTorque(transform.TransformDirection(ref_rotation * linear_control_p));
      rb.AddForce(transform.TransformDirection(ref_linear * angular_control_p));
    }
  }

  void SetReferenceVelocity(TwistStampedMsg  msg)
  {
    ref_linear.x = -(float)msg.twist.linear.y;
    ref_linear.y = (float)msg.twist.linear.z;
    ref_linear.z = (float)msg.twist.linear.x;

    /* ref_rotation.x = (float)msg.twist.angular.x; */
    /* ref_rotation.y = (float)msg.twist.angular.y; */
    /* ref_rotation.z = (float)msg.twist.angular.z; */
    ref_rotation.x = -(float)msg.twist.angular.y;
    ref_rotation.y = (float)msg.twist.angular.z;
    ref_rotation.z = (float)msg.twist.angular.x;
  }

  public void ToggleManualControl(bool b){
    enableGameControl  = b;
  }
}
