using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChaoticFlightObjectController : MonoBehaviour
{
  public GameObject managerObject;

  public float reachingDist = 2;
  bool hasGoal = false;
  public Vector3 goalPosition;
  public float thinkingPeriod = 0.5f;
  private float thinkTime = 0;

  public float moveVelocity = 10f;


  // Start is called before the first frame update
  void Start()
  {

    thinkTime = thinkingPeriod;
  }

  // Update is called once per frame
  void Update()
  {
    thinkTime += Time.deltaTime;
    if(thinkTime > thinkingPeriod){
      thinkTime -= thinkingPeriod;

      if(!hasGoal){
        goalPosition = managerObject.GetComponent<FlyingObjectAreaManager>().samplePosition();
        GetComponent<Rigidbody>().velocity = (goalPosition - transform.position).normalized * moveVelocity;
        hasGoal = true;
      }
      if(hasGoal){
        if(Vector3.Distance(goalPosition, transform.position) < reachingDist){
          hasGoal = false;
          GetComponent<Rigidbody>().velocity = Vector3.zero;
        }
      }
    }
  }

}
