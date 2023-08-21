using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;

public class LabeledZoneStringPublisher : MonoBehaviour
{
  [SerializeField] public string _topicName = "area_label";
  [SerializeField] public string areaName;

  private ROSConnection _ros;
  public StringMsg _message;
  public bool agent_inside = false;
  public List<GameObject> spawnPoints;

  // Start is called before the first frame update
  void Start()
  {
    this._ros = ROSConnection.GetOrCreateInstance();
    this._ros.RegisterPublisher<StringMsg>(this._topicName);

    // setup ROS Message
    this._message = new StringMsg();
    /* this._message.header.frame_id = this._frameId; */
    this._message.data = this.areaName;

    /* FIND SPAWNPOINTS UNDER ME */
    FindSpawnpoints();
    DisableRenderingForAll();
  }

  // Update is called once per frame
  void Update()
  {
  }

  public Transform getSpawnpointTransform(int sp_index){
    if(sp_index >= spawnPoints.Count){
      Debug.Log("ERROR! spawnpoint index out of range!");
      return null;
    }
    return spawnPoints[sp_index].GetComponent<Transform>();
  }

  // Update is called once per frame
  void OnTriggerStay(Collider col)
  {
    /* Debug.Log("SENDING ZONE INFO"); */
    agent_inside  = true;
    this._ros.Publish(this._topicName, this._message);

  }

  void FindSpawnpoints(){
    this.spawnPoints = new List<GameObject>();
    foreach (Transform child  in transform) {
      if(child.gameObject.GetComponent<BoxCollider>() == null){
        spawnPoints.Add(child.gameObject);
      }
    }
  }

  void DisableRenderingForAll(){
    foreach (Transform child  in transform) {
      child.gameObject.GetComponent<MeshRenderer>().enabled = (false);
    }
    /* GetComponent<MeshRenderer>().enabled = (false); */
  }

}
